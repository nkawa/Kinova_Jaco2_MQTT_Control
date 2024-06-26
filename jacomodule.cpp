// Kinova Gen2 / Jaco2 Python Module
// We use pybind11
#ifndef UNICODE
#define UNICODE
#endif
#include <pybind11/pybind11.h>
#include <stdio.h>
#include <iostream>
#include <dlfcn.h>
#include <vector>
#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <unistd.h>

using namespace std;
namespace py = pybind11;

// A handle to the API.
void *commandLayer_handle;

// Function pointers to the functions we need
int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MyGetGeneralInformations)(GeneralInformations &Response);
int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*MySetActiveDevice)(KinovaDevice device);

// int(*MyGetAngularCommand)(AngularPosition &);
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

KinovaDevice kinova_list[MAX_KINOVA_DEVICE];
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
            cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl
                 << endl;
            int result = (*MyInitAPI)();

            cout << "Initialization's result :" << result << endl;

            devicesCount = MyGetDevices(kinova_list, result);

            cout << "Found" << devicesCount << " Devices." << endl;

            for (int i = 0; i < devicesCount; i++)
            {
                cout << "Found a robot on the USB bus (" << kinova_list[i].SerialNumber << ")" << endl;
            }
        }
    };

    void start()
    {
        printf("JACO2 initialized\n");
    };

private:
    int programResult = 0;
};

PYBIND11_MODULE(jacomodule, m)
{
    m.doc() = "pybind11 jacomodule plugin";

    py::class_<Jaco2>(m, "Jaco2")
        .def(py::init<>())
        .def("start", &Jaco2::start);
}