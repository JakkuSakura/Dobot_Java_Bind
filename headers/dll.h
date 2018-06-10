/*
 * Real-time pose
 */
struct Pose {
    float x;
    float y;
    float z;
    float r;
    float jointAngle[4];
};

/*
 * Kinematics parameters
 */
struct Kinematics {
    float velocity;
    float acceleration;
};

/*
 * HOME related
 */
struct HOMEParams {
    float x;
    float y;
    float z;
    float r;
};

struct HOMECmd {
    uint32_t reserved;
};

struct AutoLevelingCmd {
    uint8_t controlFlag;
    float precision;
};

/*
 * Hand hold teach
 */
typedef enum HHTTrigMode {
    TriggeredOnKeyReleased,
    TriggeredOnPeriodicInterval
};

/*
 * End effector
 */
struct EndEffectorParams {
    float xBias;
    float yBias;
    float zBias;
};

/*
 * Arm orientation
 */
typedef enum ArmOrientation {
    LeftyArmOrientation,
    RightyArmOrientation,
};

/*
 * JOG related
 */
struct JOGJointParams {
    float velocity[4];
    float acceleration[4];
};

struct JOGCoordinateParams {
    float velocity[4];
    float acceleration[4];
};

struct JOGLParams {
    float velocity;
    float acceleration;
};

struct JOGCommonParams {
    float velocityRatio;
    float accelerationRatio;
};

enum {
    JogIdle,
    JogAPPressed,
    JogANPressed,
    JogBPPressed,
    JogBNPressed,
    JogCPPressed,
    JogCNPressed,
    JogDPPressed,
    JogDNPressed,
    JogEPPressed,
    JogENPressed
};

struct JOGCmd {
    uint8_t isJoint;
    uint8_t cmd;
};

/*
 * PTP related
 */
struct PTPJointParams {
    float velocity[4];
    float acceleration[4];
};

struct PTPCoordinateParams {
    float xyzVelocity;
    float rVelocity;
    float xyzAcceleration;
    float rAcceleration;
};

struct PTPLParams {
    float velocity;
    float acceleration;
};

struct PTPJumpParams {
    float jumpHeight;
    float zLimit;
};

struct PTPJump2Params {
    float startJumpHeight;
    float endJumpHeight;
    float zLimit;
};

struct PTPCommonParams {
    float velocityRatio;
    float accelerationRatio;
};

enum PTPMode {
    PTPJUMPXYZMode,
    PTPMOVJXYZMode,
    PTPMOVLXYZMode,

    PTPJUMPANGLEMode,
    PTPMOVJANGLEMode,
    PTPMOVLANGLEMode,

    PTPMOVJANGLEINCMode,
    PTPMOVLXYZINCMode,
    PTPMOVJXYZINCMode,

    PTPJUMPMOVLXYZMode,
};

struct PTPCmd {
    uint8_t ptpMode;
    float x;
    float y;
    float z;
    float r;
};

struct PTPWithLCmd {
    uint8_t ptpMode;
    float x;
    float y;
    float z;
    float r;
    float l;
};

struct ParallelOutputCmd {
    uint8_t ratio;
    uint16_t address;
    uint8_t level;
};

/*
 * CP related
 */
struct CPParams
{
    float planAcc;
    float juncitionVel;
    union {
        float acc;
        float period;
    };
    uint8_t realTimeTrack;
};

struct CPCommonParams {
    float velocityRatio;
    float accelerationRatio;
};

enum CPMode {
    CPRelativeMode,
    CPAbsoluteMode
};

struct CPCmd {
    uint8_t cpMode;
    float x;
    float y;
    float z;
    union {
        float velocity;
        float power;
    };
};

/*
 * ARC related
 */
struct ARCParams {
    float xyzVelocity;
    float rVelocity;
    float xyzAcceleration;
    float rAcceleration;
};

struct ARCCommonParams {
    float velocityRatio;
    float accelerationRatio;
};

struct ARCCmd {
    struct {
        float x;
        float y;
        float z;
        float r;
    };
    struct {
        float x;
        float y;
        float z;
        float r;
    };
};

struct CircleCmd {
    struct {
        float x;
        float y;
        float z;
        float r;
    };
    struct {
        float x;
        float y;
        float z;
        float r;
    };
    uint32_t count;
};

struct WAITCmd {
    uint32_t timeout;
};

typedef enum TRIGMode {
    TRIGInputIOMode,
    TRIGADCMode
};

typedef enum TRIGInputIOCondition {
    TRIGInputIOEqual,
    TRIGInputIONotEqual
};

typedef enum TRIGADCCondition {
    TRIGADCLT,  // Lower than
    TRIGADCLE,  // Lower than or Equal
    TRIGADCGE,  // Greater than or Equal
    TRIGADCGT   // Greater Than
};

struct TRIGCmd {
    uint8_t address;
    uint8_t mode;
    uint8_t condition;
    uint16_t threshold;
};

typedef enum IOFunction {
    IOFunctionDummy,
    IOFunctionDO,
    IOFunctionPWM,
    IOFunctionDI,
    IOFunctionADC
};

struct IOMultiplexing {
    uint8_t address;
    uint8_t multiplex;
};

struct IODO {
    uint8_t address;
    uint8_t level;
};

struct IOPWM {
    uint8_t address;
    float frequency;
    float dutyCycle;
};

struct IODI {
    uint8_t address;
    uint8_t level;
};

struct IOADC {
    uint8_t address;
    uint16_t value;
};

struct EMotor {
    uint8_t index;
    uint8_t isEnabled;
    int32_t speed;
};

struct EMotorS {
    uint8_t index;
    uint8_t isEnabled;
    int32_t speed;
    uint32_t distance;
};

/*
 * WIFI related
 */
struct WIFIIPAddress {
    uint8_t dhcp;
    uint8_t addr[4];
};

struct WIFINetmask {
    uint8_t addr[4];
};

struct WIFIGateway {
    uint8_t addr[4];
};

struct WIFIDNS {
    uint8_t addr[4];
};

/*
 * Test
 */
struct UserParams{
    float params[8];
};

/*
 * Firmware related
 */

enum FirmwareSwitchMode{
    NO_SWITCH,
    DOBOT_SWITCH,
    PRINTING_SWITCH,
    DRIVER1_SWITCH,
    DRIVER2_SWITCH,
    DRIVER3_SWITCH,
    DRIVER4_SWITCH,
    DRIVER5_SWITCH
};

struct FirmwareParams {
    uint8_t  mode;
};

enum FirewareMode{
    INVALID_MODE,
    DOBOT_MODE,
    PRINTING_MODE,
    OFFLINE_MODE
};

struct FirmwareModes {
    uint8_t  mode;
    uint8_t  ctl; //0 or 1
};

typedef enum ServoControlLoop{
    ServoPositionLoop,
    ServoVelocityLoop,
    ServoCurrentLoop
};

struct PIDParams{
    float p;
    float i;
    float d;
    float v;
    float a;
};

struct PID{
    uint8_t index;
    uint8_t controlLoop;
    PIDParams params;
};

typedef enum ColorPort{
    CL_PORT_GP1,
    CL_PORT_GP2,
    CL_PORT_GP4,
    CL_PORT_GP5
};

typedef enum InfraredPort{
    IF_PORT_GP1,
    IF_PORT_GP2,
    IF_PORT_GP4,
    IF_PORT_GP5
};

typedef enum UART4PeripheralsType {
    UART4PeripheralsUART,
    UART4PeripheralsWIFI,
    UART4PeripheralsBLE,
    UART4PeripheralsCH375
} UART4PeripheralsType;

struct PluseCmd {
    float j1;
    float j2;
    float j3;
    float j4;
    float e1;
    float e2;
} PluseCmd;

/*********************************************************************************************************
** API result
*********************************************************************************************************/
enum {
    DobotConnect_NoError,
    DobotConnect_NotFound,
    DobotConnect_Occupied
};

enum {
    DobotCommunicate_NoError,
    DobotCommunicate_BufferFull,
    DobotCommunicate_Timeout,
    DobotCommunicate_InvalidParams
};


int DobotExec(void);

int SearchDobot(char *dobotNameList, uint32_t maxLen);
int ConnectDobot(const char *portName, uint32_t baudrate, char *fwType, char *version);
int DisconnectDobot(void);

int SetCmdTimeout(uint32_t cmdTimeout);

// Device information
int SetDeviceSN(const char *deviceSN);
int GetDeviceSN(char *deviceSN, uint32_t maxLen);

int SetDeviceName(const char *deviceName);
int GetDeviceName(char *deviceName, uint32_t maxLen);

int GetDeviceVersion(uint8_t *majorVersion, uint8_t *minorVersion, uint8_t *revision);

int SetDeviceWithL(bool isWithL, bool isQueued, uint64_t *queuedCmdIndex);
int GetDeviceWithL(bool *isWithL);

int GetDeviceTime(uint32_t *deviceTime);

// Pose and Kinematics parameters are automatically get
int GetPose(Pose *pose);
int ResetPose(bool manual, float rearArmAngle, float frontArmAngle);
int GetKinematics(Kinematics *kinematics);
int GetPoseL(float *l);

// Alarms
int GetAlarmsState(uint8_t *alarmsState, uint32_t *len, uint32_t maxLen);
int ClearAllAlarmsState(void);

// HOME
int SetHOMEParams(HOMEParams *homeParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetHOMEParams(HOMEParams *homeParams);

int SetHOMECmd(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex);

int SetAutoLevelingCmd(AutoLevelingCmd *autoLevelingCmd, bool isQueued, uint64_t *queuedCmdIndex);
int GetAutoLevelingResult(float *precision);

// Handheld teach
int SetHHTTrigMode(HHTTrigMode hhtTrigMode);
int GetHHTTrigMode(HHTTrigMode *hhtTrigMode);

int SetHHTTrigOutputEnabled(bool isEnabled);
int GetHHTTrigOutputEnabled(bool *isEnabled);

int GetHHTTrigOutput(bool *isTriggered);

// EndEffector
int SetEndEffectorParams(EndEffectorParams *endEffectorParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetEndEffectorParams(EndEffectorParams *endEffectorParams);

int SetEndEffectorLaser(bool enableCtrl, bool on, bool isQueued, uint64_t *queuedCmdIndex);
int GetEndEffectorLaser(bool *isCtrlEnabled, bool *isOn);

int SetEndEffectorSuctionCup(bool enableCtrl, bool suck, bool isQueued, uint64_t *queuedCmdIndex);
int GetEndEffectorSuctionCup(bool *isCtrlEnabled, bool *isSucked);

int SetEndEffectorGripper(bool enableCtrl, bool grip, bool isQueued, uint64_t *queuedCmdIndex);
int GetEndEffectorGripper(bool *isCtrlEnabled, bool *isGripped);

// Arm orientation
int SetArmOrientation(ArmOrientation armOrientation, bool isQueued, uint64_t *queuedCmdIndex);
int GetArmOrientation(ArmOrientation *armOrientation);

// JOG functions
int SetJOGJointParams(JOGJointParams *jointJogParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetJOGJointParams(JOGJointParams *jointJogParams);

int SetJOGCoordinateParams(JOGCoordinateParams *coordinateJogParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetJOGCoordinateParams(JOGCoordinateParams *coordinateJogParams);

int SetJOGLParams(JOGLParams *jogLParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetJOGLParams(JOGLParams *jogLParams);

int SetJOGCommonParams(JOGCommonParams *jogCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetJOGCommonParams(JOGCommonParams *jogCommonParams);
int SetJOGCmd(JOGCmd *jogCmd, bool isQueued, uint64_t *queuedCmdIndex);

// PTP functions
int SetPTPJointParams(PTPJointParams *ptpJointParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetPTPJointParams(PTPJointParams *ptpJointParams);
int SetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams);
int SetPTPLParams(PTPLParams *ptpLParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetPTPLParams(PTPLParams *ptpLParams);

int SetPTPJumpParams(PTPJumpParams *ptpJumpParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetPTPJumpParams(PTPJumpParams *ptpJumpParams);
int SetPTPCommonParams(PTPCommonParams *ptpCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetPTPCommonParams(PTPCommonParams *ptpCommonParams);

int SetPTPCmd(PTPCmd *ptpCmd, bool isQueued, uint64_t *queuedCmdIndex);
int SetPTPWithLCmd(PTPWithLCmd *ptpWithLCmd, bool isQueued, uint64_t *queuedCmdIndex);

int SetPTPJump2Params(PTPJump2Params *ptpJump2Params, bool isQueued, uint64_t *queuedCmdIndex);
int GetPTPJump2Params(PTPJump2Params *ptpJump2Params);

int SetPTPPOCmd(PTPCmd *ptpCmd, ParallelOutputCmd *parallelCmd, int parallelCmdCount, bool isQueued, uint64_t *queuedCmdIndex);
int SetPTPPOWithLCmd(PTPWithLCmd *ptpWithLCmd, ParallelOutputCmd *parallelCmd, int parallelCmdCount, bool isQueued, uint64_t *queuedCmdIndex);

// CP functions
int SetCPParams(CPParams *cpParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetCPParams(CPParams *cpParams);
int SetCPCmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex);
int SetCPLECmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex);
int SetCPRHoldEnable(bool isEnable);
int GetCPRHoldEnable(bool *isEnable);
int SetCPCommonParams(CPCommonParams *cpCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetCPCommonParams(CPCommonParams *cpCommonParams);

// ARC
int SetARCParams(ARCParams *arcParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetARCParams(ARCParams *arcParams);
int SetARCCmd(ARCCmd *arcCmd, bool isQueued, uint64_t *queuedCmdIndex);
int SetCircleCmd(CircleCmd *circleCmd, bool isQueued, uint64_t *queuedCmdIndex);
int SetARCCommonParams(ARCCommonParams *arcCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
int GetARCCommonParams(ARCCommonParams *arcCommonParams);

// WAIT
int SetWAITCmd(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex);

// TRIG
int SetTRIGCmd(TRIGCmd *trigCmd, bool isQueued, uint64_t *queuedCmdIndex);

// EIO
int SetIOMultiplexing(IOMultiplexing *ioMultiplexing, bool isQueued, uint64_t *queuedCmdIndex);
int GetIOMultiplexing(IOMultiplexing *ioMultiplexing);

int SetIODO(IODO *ioDO, bool isQueued, uint64_t *queuedCmdIndex);
int GetIODO(IODO *ioDO);

int SetIOPWM(IOPWM *ioPWM, bool isQueued, uint64_t *queuedCmdIndex);
int GetIOPWM(IOPWM *ioPWM);

int GetIODI(IODI *ioDI);
int GetIOADC(IOADC *ioADC);

int SetEMotor(EMotor *eMotor, bool isQueued, uint64_t *queuedCmdIndex);
int SetEMotorS(EMotorS *eMotorS, bool isQueued, uint64_t *queuedCmdIndex);

int SetColorSensor(bool enable,ColorPort colorPort);
int GetColorSensor(uint8_t *r, uint8_t *g, uint8_t *b);

int SetInfraredSensor(bool enable,InfraredPort infraredPort);
int GetInfraredSensor(InfraredPort port, uint8_t *value);

// CAL
int SetAngleSensorStaticError(float rearArmAngleError, float frontArmAngleError);
int GetAngleSensorStaticError(float *rearArmAngleError, float *frontArmAngleError);
int SetAngleSensorCoef(float rearArmAngleCoef, float frontArmAngleCoef);
int GetAngleSensorCoef(float *rearArmAngleCoef, float *frontArmAngleCoef);

int SetBaseDecoderStaticError(float baseDecoderError);
int GetBaseDecoderStaticError(float *baseDecoderError);

int SetLRHandCalibrateValue(float lrHandCalibrateValue);
int GetLRHandCalibrateValue(float *lrHandCalibrateValue);

// WIFI
int SetWIFIConfigMode(bool enable);
int GetWIFIConfigMode(bool *isEnabled);
int SetWIFISSID(const char *ssid);
int GetWIFISSID(char *ssid, uint32_t maxLen);
int SetWIFIPassword(const char *password);
int GetWIFIPassword(char *password, uint32_t maxLen);
int SetWIFIIPAddress(WIFIIPAddress *wifiIPAddress);
int GetWIFIIPAddress(WIFIIPAddress *wifiIPAddress);
int SetWIFINetmask(WIFINetmask *wifiNetmask);
int GetWIFINetmask(WIFINetmask *wifiNetmask);
int SetWIFIGateway(WIFIGateway *wifiGateway);
int GetWIFIGateway(WIFIGateway *wifiGateway);
int SetWIFIDNS(WIFIDNS *wifiDNS);
int GetWIFIDNS(WIFIDNS *wifiDNS);
int GetWIFIConnectStatus(bool *isConnected);

//FIRMWARE
int UpdateFirmware(FirmwareParams *firmwareParams);
int SetFirmwareMode(FirmwareMode *firmwareMode);
int GetFirmwareMode(FirmwareMode *firmwareMode);

//LOSTSTEP
int SetLostStepParams(float threshold, bool isQueued, uint64_t *queuedCmdIndex);
int SetLostStepCmd(bool isQueued, uint64_t *queuedCmdIndex);

//UART4 Peripherals
int GetUART4PeripheralsType(uint8_t *type);
int SetUART4PeripheralsEnable(bool isEnable);
int GetUART4PeripheralsEnable(bool *isEnable);

//Function Pluse Mode
int SendPluse(PluseCmd *pluseCmd, bool isQueued, uint64_t *queuedCmdIndex);
int SendPluseEx(PluseCmd *pluseCmd);

// TEST
int GetUserParams(UserParams *userParams);
int GetPTPTime(PTPCmd *ptpCmd, uint32_t *ptpTime);
int GetServoPIDParams(PID *pid);
int SetServoPIDParams(PID *pid, bool isQueued, uint64_t *queuedCmdIndex);
int GetServoControlLoop(uint8_t index, uint8_t *controlLoop);
int SetServoControlLoop(uint8_t index, uint8_t controlLoop, bool isQueued, uint64_t *queuedCmdIndex);
int SaveServoPIDParams(uint8_t index, uint8_t controlLoop, bool isQueued, uint64_t *queuedCmdIndex);

// Queued command
int SetQueuedCmdStartExec(void);
int SetQueuedCmdStopExec(void);
int SetQueuedCmdForceStopExec(void);
int SetQueuedCmdStartDownload(uint32_t totalLoop, uint32_t linePerLoop);
int SetQueuedCmdStopDownload(void);
int SetQueuedCmdClear(void);
int GetQueuedCmdCurrentIndex(uint64_t *queuedCmdCurrentIndex);
