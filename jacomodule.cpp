// Kinova Gen2 / Jaco2 Python Module
// We use pybind11
#ifndef UNICODE
#define UNICODE
#endif
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdio.h>
#include <iostream>
#include <dlfcn.h>
#include <vector>
#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <unistd.h>
#include <array>

using namespace std;
namespace py = pybind11;

#define KINOVA_NO_ERR 1

// A handle to the API.
void *commandLayer_handle;

// Function pointers to the functions we need
int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MyGetGeneralInformations)(GeneralInformations &Response);
int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*MySetActiveDevice)(KinovaDevice device);

int (*MyEraseAllTrajectories)();
// int(*MyGetAngularCommand)(AngularPosition &);
int (*MySetAngularControl)();
int (*MyGetAngularPosition)(AngularPosition &);
int (*MyGetAngularCommand)(AngularPosition &);
int (*MyGetAngularVelocity)(AngularPosition &Response);
int (*MyGetAngularForce)(AngularPosition &Response);
int (*MyGetCartesianPosition)(CartesianPosition &);
int (*MyGetCartesianCommand)(CartesianPosition &);
int (*MyGetCartesianForce)(CartesianPosition &);
int (*MyMoveHome)();
int (*MySetTorqueSafetyFactor)(float factor);
int (*MySetTorqueVibrationController)(float value);
int (*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
int (*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
int (*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
int (*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
int (*MySetGravityType)(GRAVITY_TYPE Type);
int (*MyInitFingers)();
int (*MySendBasicTrajectory)(TrajectoryPoint command);
int (*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
int (*MyGetCodeVersion)(int Response[CODE_VERSION_COUNT]);
int (*MyStartForceControl)();
int (*MyStopForceControl)();
int (*MyGetEndEffectorOffset)(unsigned int *, float *, float *, float *);
int (*MySetEndEffectorOffset)(unsigned int status, float x, float y, float z);
int (*MyGetProtectionZone)(ZoneList &Response);
int (*MyEraseAllProtectionZones)();
int (*MySetProtectionZone)(ZoneList Command);
int (*MySetCartesianControl)();
int (*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &Response);
int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
int (*MySetPositionLimitDistance)(float Command[COMMAND_SIZE]);
int (*MySetActuatorPID)(unsigned int address, float P, float I, float D);
int (*MyRefresDevicesList)();

KinovaDevice k_list[MAX_KINOVA_DEVICE];
int devicesCount;

class Jaco2
{
public:
    Jaco2()
    {
        printf("Jaco2 Created. DLL initializing\n");

        // We load the API
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);

        // We load the functions from the library
        MyInitAPI = (int (*)())dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int (*)())dlsym(commandLayer_handle, "CloseAPI");

        MyGetGeneralInformations = (int (*)(GeneralInformations &info))dlsym(commandLayer_handle, "GetGeneralInformations");

        MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result))dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int (*)(KinovaDevice devices))dlsym(commandLayer_handle, "SetActiveDevice");
        MySetAngularControl = (int (*)())dlsym(commandLayer_handle, "SetAngularControl");
        MyGetAngularCommand = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularCommand");
        MyGetAngularPosition = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularPosition");
        MyGetAngularVelocity = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularVelocity");
        MyGetAngularForce = (int (*)(AngularPosition &Response))dlsym(commandLayer_handle, "GetAngularForce");
        MyGetCartesianPosition = (int (*)(CartesianPosition &))dlsym(commandLayer_handle, "GetCartesianPosition");
        MyGetCartesianCommand = (int (*)(CartesianPosition &))dlsym(commandLayer_handle, "GetCartesianCommand");
        MyGetCartesianForce = (int (*)(CartesianPosition &))dlsym(commandLayer_handle, "GetCartesianForce");
        MyMoveHome = (int (*)())dlsym(commandLayer_handle, "MoveHome");

        MySetTorqueSafetyFactor = (int (*)(float))dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        MySetTorqueVibrationController = (int (*)(float))dlsym(commandLayer_handle, "SetTorqueVibrationController");
        MySwitchTrajectoryTorque = (int (*)(GENERALCONTROL_TYPE))dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        MySetTorqueControlType = (int (*)(TORQUECONTROL_TYPE))dlsym(commandLayer_handle, "SetTorqueControlType");
        MyRunGravityZEstimationSequence = (int (*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]))dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
        MySetGravityOptimalZParam = (int (*)(float Command[GRAVITY_PARAM_SIZE]))dlsym(commandLayer_handle, "SetGravityOptimalZParam");
        MySetGravityType = (int (*)(GRAVITY_TYPE Type))dlsym(commandLayer_handle, "SetGravityType");
        MyInitFingers = (int (*)())dlsym(commandLayer_handle, "InitFingers");

        MySendBasicTrajectory = (int (*)(TrajectoryPoint))dlsym(commandLayer_handle, "SendBasicTrajectory");
        MySendAngularTorqueCommand = (int (*)(float Command[COMMAND_SIZE]))dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        MyGetCodeVersion = (int (*)(int Response[CODE_VERSION_COUNT]))dlsym(commandLayer_handle, "GetCodeVersion");
        MyStartForceControl = (int (*)())dlsym(commandLayer_handle, "StartForceControl");
        MyStopForceControl = (int (*)())dlsym(commandLayer_handle, "StopForceControl");
        MyGetEndEffectorOffset = (int (*)(unsigned int *, float *, float *, float *))dlsym(commandLayer_handle, "GetEndEffectorOffset");
        MySetEndEffectorOffset = (int (*)(unsigned int, float, float, float))dlsym(commandLayer_handle, "SetEndEffectorOffset");
        MyGetProtectionZone = (int (*)(ZoneList &))dlsym(commandLayer_handle, "GetProtectionZone");
        MyEraseAllProtectionZones = (int (*)())dlsym(commandLayer_handle, "EraseAllProtectionZones");
        MySetProtectionZone = (int (*)(ZoneList))dlsym(commandLayer_handle, "SetProtectionZone");
        MySetCartesianControl = (int (*)())dlsym(commandLayer_handle, "SetCartesianControl");
        MyGetGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &Response))dlsym(commandLayer_handle, "GetGlobalTrajectoryInfo");
        MySendAdvanceTrajectory = (int (*)(TrajectoryPoint))dlsym(commandLayer_handle, "SendAdvanceTrajectory");
        MySetPositionLimitDistance = (int (*)(float Command[COMMAND_SIZE]))dlsym(commandLayer_handle, "SetPositionLimitDistance");
        MySetActuatorPID = (int (*)(unsigned int, float, float, float))dlsym(commandLayer_handle, "SetActuatorPID");
        MyGetGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &Response))dlsym(commandLayer_handle, "GetGlobalTrajectoryInfo");
        MyRefresDevicesList = (int (*)())dlsym(commandLayer_handle, "RefresDevicesList");

        //        MyEraseAllTrajectories = (int (*)())dlsym(commandLayer_handle, "EraseAllTrajectories");
        //
        //        MySendJoystickCommand

        // Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
            (MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
            (MyMoveHome == NULL) || (MyInitFingers == NULL))
        {
            cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
            programResult = 0;
        }
        else
        {

            if ((MySetAngularControl == NULL)) //|| (MyEraseAllTrajectories == NULL))
            {
                cout << "Cant initialize AngularControl or EraseAllTrajectories" << endl;
            }

            cout << "Kinova API Initialized" << endl;
            int result = (*MyInitAPI)();
            //            cout << "Initialization's result :" << result << endl;
            devicesCount = MyGetDevices(k_list, result);
            cout << "Found " << devicesCount << " Devices." << endl;
            for (int i = 0; i < devicesCount; i++)
            {
                cout << i + 1 << ":Robot on USB:" << k_list[i].SerialNumber << ", " << k_list[i].Model << " Type:" << k_list[i].DeviceType << " Ver: ";
                cout << k_list[i].VersionMajor << "." << k_list[i].VersionMinor << "." << k_list[i].VersionRelease << endl;
            }
            if (devicesCount > 0)
            { // we just use 1st device for this lib.
                MySetActiveDevice(k_list[0]);
                cout << "Device " << 1 << " activated" << endl;
            }
        }
    };

    void start()
    {
        printf("JACO2 initialized\n");

        int ret = MyInitFingers();
        printf("Init Finger %d\n", ret);
    };

    void moveHome()
    {
        int ret = MyMoveHome();
        printf("My MoveHome %d\n", ret);
    }

    std::array<float, 6> getCartesianPoint()
    {
        CartesianPosition cp;
        cp.InitStruct(); // initialize

        int ret = MyGetCartesianPosition(cp);
        cout << "Get Cartesian: " << ret << endl;
        std::array<float, 6> coord =
            {cp.Coordinates.X,
             cp.Coordinates.Y,
             cp.Coordinates.Z,
             cp.Coordinates.ThetaX,
             cp.Coordinates.ThetaY,
             cp.Coordinates.ThetaZ};

        if (ret != KINOVA_NO_ERR)
        {
            cout << "Umm Errr on Get Cartesian" << endl;
            cp.Coordinates.X = -25555;
        }

        //        cout << "Coord:" << coord << endl;
        // 本当は CartesianPosition を返したい

        return coord;
    }
    // Python 経由でセンサーデータを取得

    int setCartesianControl()
    {
        return MySetCartesianControl();
    }

    int setAngularControl()
    {
        return MySetAngularControl();
    }

    int eraseAllTrajectories()
    {
        //        return MyEraseAllTrajectories();
        return -1;
    }

    int sendTrajectory(const std::array<float, 6> &coord)
    {
        TrajectoryPoint tp;
        tp.InitStruct();
        tp.Position.Type = CARTESIAN_POSITION; // set Position
        CartesianInfo *ci = &tp.Position.CartesianPosition;

        ci->X = coord[0];
        ci->Y = coord[1];
        ci->Z = coord[2];
        ci->ThetaX = coord[3];
        ci->ThetaY = coord[4];
        ci->ThetaZ = coord[5];

        //        cout << "Got Floats! " << coord[0] << endl;
        int ret = MySendBasicTrajectory(tp);

        return ret;
    }

    int sendAngleTrajectory(const std::array<float, 6> &coord)
    {
        TrajectoryPoint tp;
        tp.InitStruct();
        tp.Position.Type = ANGULAR_POSITION; // set Position
        AngularInfo *ai = &tp.Position.Actuators;

        ai->Actuator1 = coord[0];
        ai->Actuator2 = coord[1];
        ai->Actuator3 = coord[2];
        ai->Actuator4 = coord[3];
        ai->Actuator5 = coord[4];
        ai->Actuator6 = coord[5];

        //        cout << "Got Floats! " << coord[0] << endl;
        //        cout << "Got Floats! " << coord[0] << endl;
        int ret = MySendAdvanceTrajectory(tp);
        //        cout << "Send Angular Position " << ret << endl;
        return ret;
    }

    std::array<float, 6> getAngularPosition()
    {
        //        MyGetAngularCommand
        AngularPosition ap;
        ap.InitStruct(); // may be share ..
        int ret = MyGetAngularPosition(ap);

        if (ret != KINOVA_NO_ERR)
        {
            cout << "Errr on Get AngularPosition" << endl;
            ap.Actuators.Actuator1 = -25555;
        }

        std::array<float, 6> angle =
            {ap.Actuators.Actuator1,
             ap.Actuators.Actuator2,
             ap.Actuators.Actuator3,
             ap.Actuators.Actuator4,
             ap.Actuators.Actuator5,
             ap.Actuators.Actuator6};

        return angle;
    }

    std::array<float, 6> getAngularCommand()
    {
        //        MyGetAngularCommand
        AngularPosition ap;
        ap.InitStruct(); // may be share ..
        int ret = MyGetAngularCommand(ap);

        if (ret != KINOVA_NO_ERR)
        {
            cout << "Errr on Get AngularCommand" << endl;
            ap.Actuators.Actuator1 = -25555;
        }

        std::array<float, 6> angle =
            {ap.Actuators.Actuator1,
             ap.Actuators.Actuator2,
             ap.Actuators.Actuator3,
             ap.Actuators.Actuator4,
             ap.Actuators.Actuator5,
             ap.Actuators.Actuator6};

        return angle;
    }

    // not yet implemented
    int sendAngularTorqueCommand(std::array<float, 6>)
    {
        float command[70];

        int ret = MySendAngularTorqueCommand(command);
        return ret;
    }

private:
    int programResult = 0;
};

PYBIND11_MODULE(jacomodule, m)
{
    m.doc() = "pybind11 jacomodule plugin";

    py::class_<Jaco2>(m, "Jaco2")
        .def(py::init<>())
        .def("start", &Jaco2::start)
        .def("moveHome", &Jaco2::moveHome, "Move to Home")
        .def("getCartesianPoint", &Jaco2::getCartesianPoint, "A function that returns current Jaco2 coordinates")
        .def("setCartesianControl", &Jaco2::setCartesianControl, "Set Cartesian Control")
        .def("setAngularControl", &Jaco2::setAngularControl, "Set Angular Control")
        .def("eraseAllTrajectories", &Jaco2::eraseAllTrajectories, "erase All Trajectories")
        .def("getAngularPosition", &Jaco2::getAngularPosition, "A function that returns current Jaco2 angles")
        .def("getAngularCommand", &Jaco2::getAngularCommand, "A function that returns current Jaco2 angle command")
        .def("sendTrajectory", &Jaco2::sendTrajectory, "A function that sends coordinates")
        .def("sendAngleTrajectory", &Jaco2::sendAngleTrajectory, "A function that sends angles");
}