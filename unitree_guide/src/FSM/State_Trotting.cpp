/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

}

State_Trotting::~State_Trotting(){
    delete _gait;
}

void State_Trotting::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();      // Tính toán các lệnh di chuyển dựa trên lệnh từ người dùng và trạng thái hiện tại của robot
                    // Tính toán _vCmdGlobal, _wCmdGlobal, _pcd, _Rd,

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);      // Lệnh này thiết lập các tham số cần thiết cho bước đi (gait) của robot.
                                                                                // _vCmdGlobal.segment(0,2):    vận tốc di chuyển của robot trên các trục X và Y
                                                                                // _wCmdGlobal(2):              vận tốc góc của robot quanh trục Z
                                                                                // _gaitHeight:                 chiều cao của bước đi

    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);     // tính toán vị trí và vận tốc của 4 bàn chân robot dựa trên các thông số như vận tốc x, y và delta yaw được tính toán trước đó trong hàm setGait
                                                            // _posFeetGlobalGoal: Vị trí mục tiêu của 4 bàn chân trong hệ tọa độ toàn cục. 
                                                            // _velFeetGlobalGoal: Vận tốc mục tiêu của 4 bàn chân trong hệ tọa độ toàn cục.


    // Tính toán mô-men xoắn (torque) cần thiết cho các khớp chân của robot để đạt được vị trí và vận tốc mong muốn của bàn chân. 
    //      Dựa trên các lỗi vị trí và vận tốc, hàm này xác định lực cần thiết để điều khiển chân robot, sau đó chuyển đổi lực này thành mô-men xoắn trên các khớp.
    calcTau();
    // Tính toán góc khớp (joint angles) và vận tốc khớp (joint velocities) mong muốn cho từng khớp của robot dựa trên vị trí và vận tốc bàn chân.
    calcQQd();
    // Kết quả từ hàm calcTau() không trực tiếp sử dụng trong hàm calcQQd(). Hai hàm này hoạt động độc lập và phục vụ các mục đích khác nhau:


    if(checkStepOrNot()){               // Kiểm tra xem robot có cần bước chân tiếp theo hay không
        _ctrlComp->setStartWave();      // Nếu có, đặt trạng thái cho bước chân tiếp theo
    }else{
        _ctrlComp->setAllStance();      // Nếu không, đặt trạng thái cho tất cả các chân đều ở trạng thái đứng yên
    }

    _lowCmd->setTau(_tau);                      // Đặt mô-men vào low level control
    _lowCmd->setQ(vec34ToVec12(_qGoal));        // Đặt góc khớp vào low level control
    _lowCmd->setQd(vec34ToVec12(_qdGoal));      // Đặt vận tốc góc khớp vào low level control

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){            // Nếu chân i không tiếp xúc với mặt đất
            _lowCmd->setSwingGain(i);       // Đặt hệ số cho chế độ swing
        }else{
            _lowCmd->setStableGain(i);      // Nếu chân tiếp đất, đặt hệ số cho chế độ ổn định
        }
    }

}

bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

// Tính toán _vCmdGlobal, _wCmdGlobal, _pcd, _Rd
void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;      // Chuyển đổi vận tốc cơ thể (_vCmdBody) từ hệ tọa độ gắn với cơ thể (Body frame) 
                                                // sang hệ tọa độ toàn cục (Global frame) thông qua ma trận quay _B2G_RotMat (Body-to-Global rotation matrix). 
                                                // Kết quả được lưu vào _vCmdGlobal.

    // Giới hạn giá trị vận tốc toàn cục trên các trục X và Y bằng cách sử dụng hàm saturation. 
    // Vận tốc _vCmdGlobal được giữ trong khoảng từ (_velBody - 0.2) đến (_velBody + 0.2) để đảm bảo robot không thay đổi vận tốc quá nhanh hoặc chậm.
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    // Tính toán vị trí mục tiêu (_pcd) của robot bằng cách cộng vận tốc toàn cục (_vCmdGlobal) nhân với khoảng thời gian bước (delta time - _ctrlComp->dt).
    // Sau đó, giới hạn vị trí mục tiêu để đảm bảo không vượt quá phạm vi cho phép của vị trí cơ thể hiện tại (từ _posBody - 0.05 đến _posBody + 0.05).
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    // Đặt vận tốc trên trục Z (chiều cao) về 0, do không cần điều chỉnh vận tốc theo chiều cao trong trạng thái trotting (chạy trên mặt phẳng).
    _vCmdGlobal(2) = 0;

    /* Turning */
    // Tính toán lệnh quay (yaw) của robot bằng cách cộng thêm tốc độ quay (_dYawCmd) nhân với khoảng thời gian bước (_ctrlComp->dt). 
    // Điều này làm cho góc quay yaw được cập nhật theo thời gian.
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    // Tạo ra ma trận quay _Rd dựa trên góc quay yaw mới (_yawCmd). Ma trận này biểu diễn sự quay quanh trục Z (yaw rotation).
    _Rd = rotz(_yawCmd);
    // Đặt tốc độ quay (yaw rate) vào _wCmdGlobal trên trục Z. Điều này cho phép điều khiển tốc độ quay của robot theo trục yaw.
    _wCmdGlobal(2) = _dYawCmd;
}

void State_Trotting::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Trotting::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

