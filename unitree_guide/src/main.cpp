/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

// bất kỳ đoạn mã nào trong code được bảo vệ bởi #ifdef <VARIABLE_NAME> sẽ được biên dịch 
// nếu biến VARIABLE_NAME đã được định nghĩa trong CMakelists.txt bằng lệnh add_definitions(-D<VARIABLE_NAME>)

#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    /* Thiết lập tiến trình để chạy trong chế độ real-time */
    setProcessScheduler();
    /* Định dạng đầu ra của console với 3 chữ số thập phân */
    std::cout << std::fixed << std::setprecision(3);

#ifdef RUN_ROS
    ros::init(argc, argv, "unitree_gazebo_servo"); // Khởi tạo node ROS với tên 'unitree_gazebo_servo'
#endif // RUN_ROS

    IOInterface *ioInter;  // Con trỏ tới đối tượng giao diện I/O (sẽ phụ thuộc vào chế độ thực hay mô phỏng)
    CtrlPlatform ctrlPlat;  // Biến để lưu nền tảng điều khiển (GAZEBO hoặc REALROBOT)

#ifdef COMPILE_WITH_SIMULATION
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);     // Khởi tạo các thành phần điều khiển của robot với giao diện I/O đã chọn
    ctrlComp->ctrlPlatform = ctrlPlat;                          // Thiết lập nền tảng điều khiển
    ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->running = &running;                               // Liên kết biến cờ `running` để dừng chương trình khi cần

#ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();
#endif
#ifdef ROBOT_TYPE_Go1
    ctrlComp->robotModel = new Go1Robot();
#endif
    // Trot (đi bằng cách di chuyển hai chân chéo nhau), đi bước kiệu
    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot         // Khởi tạo bộ sinh sóng cho dáng đi (gait) của robot
    // Crawl (bò)
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // Pronk (bước nhảy bật cả 4 chân)
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    ctrlComp->geneObj();                    // Khởi tạo các đối tượng khác liên quan đến thành phần điều khiển

    ControlFrame ctrlFrame(ctrlComp);       // Khởi tạo khung điều khiển với các thành phần điều khiển đã được cấu hình
                                            // Lớp ControlFrame có khả năng điều khiển toàn bộ hệ thống robot, 
                                            // và nó chứa logic để thực hiện các bước điều khiển trong mỗi chu kỳ.

    signal(SIGINT, ShutDown);               // Đăng ký tín hiệu SIGINT (Ctrl+C) để dừng chương trình một cách an toàn

    while (running)
    {
        /*
        mỗi chu kỳ trong vòng lặp (với tần số 500 Hz), 
        robot sẽ thực hiện các bước điều khiển mới dựa trên 
        trạng thái hiện tại của các thành phần trong ctrlComp 
        và môi trường tương tác (robot thực hoặc mô phỏng)
        */
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
