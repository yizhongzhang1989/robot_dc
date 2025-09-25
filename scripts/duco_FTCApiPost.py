import requests
import json
import time

# post json command to FTC
def FTC_ApiCommand(url,params,headers,timeout):
    try:
        # use post(json=) here, not get(params=)
        response = requests.post(url, data=params, headers=headers,timeout=timeout)

        # print("Status Code:", response.status_code)
        # print("Response Body:", response.text)
        return response
    except requests.Timeout:
        print("Request Timeout")
    except requests.ConnectionError:
        print("Can not connect")
    except requests.RequestException as e:
        print("An error occurred:", e)
        return None

#============================FTC Process=================================

# start FTC
def FTC_start():
    url_start = "http://192.168.1.20:8080/start"
    params_start = {"CmdName": "start"}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_start, params_start, headers, 5)
    return response

# stop FTC
def FTC_stop():
    url_stop = "http://192.168.1.20:8080/stop"
    params_stop = {"CmdName": "stop"}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_stop, params_stop, headers, 5)
    return response

# set FTC index
def FTC_SetIndex(index):
    url_setFTSetIndex = "http://192.168.1.20:8080/setFTSetIndex"
    params_setFTSetIndex = {"CmdName": "setFTSeTIndex", "CmdData": [index]}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setFTSetIndex, json.dumps(params_setFTSetIndex), headers,5)
    return response

# set FTC DK assembly flag
def FTC_SetDKAssemFlag(flag):
    url_setDKAssemFlag = "http://192.168.1.20:8080/setDKAssemFlag"
    params_setDKAssemFlag = {"CmdName": "setDKAssemFlag", "CmdData": [flag]}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setDKAssemFlag, json.dumps(params_setDKAssemFlag), headers,5)
    return response

# set FTC force value of current program
def FTC_setFTValue(force):
    # input force is a list of 6 values, e.g. [0, 0, 0, 0, 0, 0]
    url_setFTValue = "http://192.168.1.20:8080/setFTValue"
    params_setFTValue = {"CmdName": "setFTValue","FTSetName": "default","CmdData": force}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setFTValue, json.dumps(params_setFTValue), headers, 5)
    return response

# set FTC force value of current program in real-time
def FTC_setFTValueRT(force):
    # input force is a list of 6 values, e.g. [0, 0, 0, 0, 0, 0]
    url_setFTValueRT = "http://192.168.1.20:8080/setFTValueRT"
    params_setFTValueRT = {"CmdName": "setFTValueRT", "CmdData": force}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setFTValueRT, json.dumps(params_setFTValueRT), headers, 5)
    return response

# set All FTC parameters of current programs
def FTC_setFTValueAll(isProgram,ftcProgram,onlyMonitor,graCalcIndex,ftEnabled,ftSet,dead_zone,disEndLimit,angleEndLimit,timeEndLimit,ftEndLimit,disAng6D_EndLimit,ftcEndType,quickSetIndex,B,M,vel_limit,cor_pos_limit,maxForce_1,ifDKStopOnMaxForce_1,ifRobotStopOnMaxForce_1,maxForce_2,ifDKStopOnMaxForce_2,ifRobotStopOnMaxForce_2,ifDKStopOnTimeDisMon,ifRobotStopOnTimeDisMon,ifNeedInit,withGroup,ftcSetGroup,ignoreSensor):
    url_setFTValueAll = "http://192.168.1.20:8080/setFTValueAll"
    params_setFTValueAll = {"CmdName": "setFTSetAll",
                            "FTSetName": "default",
                            "ftSetJson": {"isProgram": isProgram,
                                          "ftcProgram": ftcProgram,
                                          "onlyMonitor": onlyMonitor,
                                          "graCalcIndex": graCalcIndex,
                                          "ftEnabled": ftEnabled,
                                          "ftSet": ftSet,
                                          "dead_zone": dead_zone,
                                          "disEndLimit": disEndLimit,
                                          "angleEndLimit":angleEndLimit,
                                          "timeEndLimit": timeEndLimit,
                                          "ftEndLimit": ftEndLimit,
                                          "disAng6D_EndLimit": disAng6D_EndLimit,
                                          "ftcEndType": ftcEndType,
                                          "quickSetIndex": quickSetIndex,
                                          "B": B,
                                          "M": M,
                                          "vel_limit": vel_limit,
                                          "cor_pos_limit": cor_pos_limit,
                                          "maxForce_1": maxForce_1,
                                          "ifDKStopOnMaxForce_1": ifDKStopOnMaxForce_1,
                                          "ifRobotStopOnMaxForce_1": ifRobotStopOnMaxForce_1,
                                          "maxForce_2": maxForce_2,
                                          "ifDKStopOnMaxForce_2":ifDKStopOnMaxForce_2,
                                          "ifRobotStopOnMaxForce_2": ifRobotStopOnMaxForce_2,
                                          "ifDKStopOnTimeDisMon": ifDKStopOnTimeDisMon,
                                          "ifRobotStopOnTimeDisMon": ifRobotStopOnTimeDisMon,
                                          "name": "default",
                                          "ifNeedInit": ifNeedInit,
                                          "withGroup": withGroup,
                                          "ftcSetGroup": ftcSetGroup,
                                          "ignoreSensor": ignoreSensor
                                          },
                            }
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setFTValueAll, json.dumps(params_setFTValueAll), headers, 5)
    return response

# set All FTC parameters of current programs in real-time
def FTC_setFTsetAllRT(isProgram,ftcProgram,onlyMonitor,graCalcIndex,ftEnabled,ftSet,dead_zone,disEndLimit,angleEndLimit,timeEndLimit,ftEndLimit,disAng6D_EndLimit,ftcEndType,quickSetIndex,B,M,vel_limit,cor_pos_limit,maxForce_1,ifDKStopOnMaxForce_1,ifRobotStopOnMaxForce_1,maxForce_2,ifDKStopOnMaxForce_2,ifRobotStopOnMaxForce_2,ifDKStopOnTimeDisMon,ifRobotStopOnTimeDisMon,ifNeedInit,withGroup,ftcSetGroup,ignoreSensor):
    url_setFTSetAllRT = "http://192.168.1.20:8080/setFTValueAllRT"
    params_setFTSetAllRT = {"CmdName": "setFTSetAllRT",
                            "ftSetJson": {"isProgram": isProgram,
                                          "ftcProgram": ftcProgram,
                                          "onlyMonitor": onlyMonitor,
                                          "graCalcIndex": graCalcIndex,
                                          "ftEnabled": ftEnabled,
                                          "ftSet": ftSet,
                                          "dead_zone": dead_zone,
                                          "disEndLimit": disEndLimit,
                                          "angleEndLimit":angleEndLimit,
                                          "timeEndLimit": timeEndLimit,
                                          "ftEndLimit": ftEndLimit,
                                          "disAng6D_EndLimit": disAng6D_EndLimit,
                                          "ftcEndType": ftcEndType,
                                          "quickSetIndex": quickSetIndex,
                                          "B": B,
                                          "M": M,
                                          "vel_limit": vel_limit,
                                          "cor_pos_limit": cor_pos_limit,
                                          "maxForce_1": maxForce_1,
                                          "ifDKStopOnMaxForce_1": ifDKStopOnMaxForce_1,
                                          "ifRobotStopOnMaxForce_1": ifRobotStopOnMaxForce_1,
                                          "maxForce_2": maxForce_2,
                                          "ifDKStopOnMaxForce_2":ifDKStopOnMaxForce_2,
                                          "ifRobotStopOnMaxForce_2": ifRobotStopOnMaxForce_2,
                                          "ifDKStopOnTimeDisMon": ifDKStopOnTimeDisMon,
                                          "ifRobotStopOnTimeDisMon": ifRobotStopOnTimeDisMon,
                                          "name": "default",
                                          "ifNeedInit": ifNeedInit,
                                          "withGroup": withGroup,
                                          "ftcSetGroup": ftcSetGroup,
                                          "ignoreSensor": ignoreSensor
                                          },
                            }
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setFTSetAllRT, json.dumps(params_setFTSetAllRT), headers, 5)
    return response

# get FTC cycle force values
def FTC_getCycleFTValue():
    url_getCycleFTValue = "http://192.168.1.20:8080/getCycleFTValue"
    params_getCycleFTValue = {"CmdName": "getCycleFTValue","FTSetName": "","CmdData": [0]}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_getCycleFTValue, json.dumps(params_getCycleFTValue), headers, 5)

    data = response.json()
    ftMin = data["ftMin"]
    ftMax = data["ftMax"]
    return  ftMin, ftMax

# get FTC force value of current program
def FTC_getFTValue():
    url_getFTValue = "http://192.168.1.20:8080/getFTValue"
    params_getFTValue = {"CmdName": " getFTValue","FTSetName": "default","CmdData": [0]}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_getFTValue, json.dumps(params_getFTValue), headers, 5)

    data = response.json()
    ftset_value = data["fTSetValue"]
    return ftset_value

# get All FTC parameters of current program
def FTC_getFTValueAll():
    url_getFTValueAll = "http://192.168.1.20:8080/getFTValueAll"
    params_getFTValueAll = {"CmdName": " getFTValueAll","FTSetName": "default","CmdData": [0]}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_getFTValueAll, json.dumps(params_getFTValueAll), headers, 5)
    return response.json()

# get FTC process flags
def FTC_getFTFlag():
    # default Flag_ok = True, when stop, Flag_ok will be False
    url_getFTValue = "http://192.168.1.20:8080/getFTFlag"
    params_getFTValue = {"CmdName": " getFTFlag"}
    headers = {'Content-Type': 'application/json'}
    try:
        response = FTC_ApiCommand(url_getFTValue, json.dumps(params_getFTValue), headers, 5)
        try:
            data = response.json()
        except json.JSONDecodeError as e:
            print("response.json() decode failed:", e)
            return False, False, False, False
        
        message_str = data.get("message", "")
        if not message_str:
            print("message field is empty")
            return False, False, False, False
        try:
            message_dict = json.loads(message_str)
        except json.JSONDecodeError as e:
            print("message_str is not valid JSON:", e)
            return False, False, False, False

        flag1 = message_dict.get("DK_ASSEM_OK_Flag_FromAPI", False)
        flag2 = message_dict.get("DK_ASSEM_Force_Monitor_Flag_1", False)
        flag3 = message_dict.get("DK_ASSEM_Force_Monitor_Flag_2", False)
        flag4 = message_dict.get("DK_ASSEM_Time_Distance_Monitor_Flag_1", False)
        return flag1, flag2, flag3, flag4

    except Exception as e:
        print("FTC_getFTFlag call exception:", e)
        return False, False, False, False

# set FTC Max Force 1 value
def FTC_setFTMax_1_Value(FTMax_1_Value):
    url_setFTMax_1_Value = "http://192.168.1.20:8080/setFTMax_1_Value"
    params_setFTMax_1_Value = {"CmdName": "setFTMax_1_Value","FTSetName": "default","CmdData": FTMax_1_Value}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setFTMax_1_Value, json.dumps(params_setFTMax_1_Value), headers, 5)
    return response.json()

# set FTC Max Force 2 value
def FTC_setFTMax_2_Value(FTMax_2_Value):
    url_setFTMax_2_Value = "http://192.168.1.20:8080/setFTMax_2_Value"
    params_setFTMax_2_Value = {"CmdName": "setFTMax_2_Value","FTSetName": "default","CmdData": FTMax_2_Value}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_setFTMax_2_Value, json.dumps(params_setFTMax_2_Value), headers, 5)
    return response.json()

#============================FTC Process End=================================

#============================Load identification=============================
# add FTC GraCalc
def FTC_addGraCalc(GraCalName):
    # GraCalName is a string, e.g. "one3"
    url_addGraCalc = "http://192.168.1.20:8080/addGraCalc"
    params_addGraCalc = {"CmdName": "addGraCalc","GraCalName": GraCalName}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_addGraCalc, json.dumps(params_addGraCalc), headers, 5)
    return

# delete FTC GraCalc
def FTC_delGraCalc():
    url_delGraCalc = "http://192.168.1.20:8080/delGraCalc"
    params_delGraCalc = {"CmdName": "delGraCalc"}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_delGraCalc, json.dumps(params_delGraCalc), headers, 5)
    return

# activate FTC GraCalc
def FTC_activateGraCalc(GraCalName):
    # GraCalName is a string, e.g. "one3"
    url_activateGraCalc = "http://192.168.1.20:8080/activateGraCalc"
    params_activateGraCalc = {"CmdName": "activateGraCalc","GraCalName": GraCalName}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_activateGraCalc, json.dumps(params_activateGraCalc), headers, 5)
    return

# get FTC GraCalc
def FTC_getGraCalc(GraCalName):
    # GraCalName is a string, e.g. "Ser_USB"
    url_getGraCalc = "http://192.168.1.20:8080/getGraCalc"
    params_getGraCalc = {"CmdName": "getGraCalc","GraCalName": GraCalName}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_getGraCalc, json.dumps(params_getGraCalc), headers, 5)
    return response.json()

# set FTC sensor installation
def FTC_setSensorInstall(CmdData):
    # CmdData is a list of 6 values, e.g. [0, 0, 0, 0, 0, 0]
    url_setSensorInstall = "http://192.168.1.20:8080/setSensorInstall"
    params_setSensorInstall = {"CmdName": "setSensorInstall","CmdData": CmdData}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_setSensorInstall, json.dumps(params_setSensorInstall), headers, 5)
    return

# set FTC target force point
def FTC_setTargetForcePoint(TargetForcePoint):
    # TargetForcePoint is a list of 6 values, e.g. [0, 0, 0, 0, 0, 0]
    url_setTargetForcePoint = "http://192.168.1.20:8080/setTargetForcePoint"
    params_setTargetForcePoint = {"CmdName": "setTargetForcePoint","CmdData": TargetForcePoint}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_setTargetForcePoint, json.dumps(params_setTargetForcePoint), headers, 5)
    return

# set FTC target force point in real-time
def FTC_setTargetForcePointRT(TargetForcePoint):
    # TargetForcePoint is a list of 6 values, e.g. [0, 0, 0, 0, 0, 0]
    url_setTargetForcePointRT = "http://192.168.1.20:8080/setTargetForcePointRT"
    params_setTargetForcePointRT = {"CmdName": "setTargetForcePointRT","CmdData": TargetForcePoint}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_setTargetForcePointRT, json.dumps(params_setTargetForcePointRT), headers, 5)
    return

# record FTC GraCalc point
def FTC_recGraCalcPoint(ponitindex):
    url_recGraCalcPoint = "http://192.168.1.20:8080/recGraCalcPoint"
    params_recGraCalcPoint = {"CmdName": "recGraCalcPoint","PointIndex": ponitindex}
    headers = {'Content-Type': 'application/json'}
    response = FTC_ApiCommand(url_recGraCalcPoint, json.dumps(params_recGraCalcPoint), headers, 5)
    return response.json()

# make FTC GraCalc
def FTC_doGraCalc():
    url_doGraCalc = "http://192.168.1.20:8080/doGraCalc"
    params_doGraCalc = {"CmdName": "doGraCalc"}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_doGraCalc, json.dumps(params_doGraCalc), headers, 5)
    return

# save FTC GraCalc
def FTC_saveGraCalc():
    url_saveGraCalc = "http://192.168.1.20:8080/saveGraCalc"
    params_saveGraCalc = {"CmdName": "saveGraCalc"}
    headers = {'Content-Type': 'application/json'}
    FTC_ApiCommand(url_saveGraCalc, json.dumps(params_saveGraCalc), headers, 5)
    return

# ============================Load identification End=========================

def main():

    FTC_start()
    print("FTC started")
    
    # FTC_SetIndex(19)
    # print("FTC Index set to 19")

    # FTC_SetDKAssemFlag(0)
    # print("FTC DK Assembly Flag set to 0")

    # FTC_getCycleFTValue()
    # print("FTC Cycle FT Value retrieved")

    # ftset_value = FTC_getFTValue()
    # print("Current FTC Set Value:", ftset_value)

    # response = FTC_getFTValueAll()
    # print("Current FTC Set Value All:", response)

    # Flag1,Flag2,Flag3,Flag4 = FTC_getFTFlag()
    # print("FTC Flags:", Flag1, Flag2, Flag3, Flag4)

    # data = FTC_getGraCalc("Ser_USB")
    # print("FTC GraCalc Data:", data)

    #----------zero force drag mode open------------
    isProgram=False
    ftcProgram=None
    onlyMonitor=False
    graCalcIndex=5
    ftEnabled=[True,True,True,True,True,True]
    ftSet=[0,0,0,0,0,0]
    dead_zone=[1,1,1,0.1,0.1,0.1]
    disEndLimit=5000
    timeEndLimit=60
    ftEndLimit=[0,0,0,0,0,0]
    disAng6D_EndLimit=[0,0,0,0,0,0]
    ftcEndType=6
    quickSetIndex=[0,0,0,0,0,0]
    B=[6000,6000,6000,4500,4500,4500]
    M=[20,20,20,25,25,25]
    vel_limit=[500,500,500,500,500,500]
    cor_pos_limit=[10,10,10,5,5,5]
    maxForce_1=[0,0,0,0,0,0]
    ifDKStopOnMaxForce_1=False
    ifRobotStopOnMaxForce_1=False
    maxForce_2=[0,0,0,0,0,0]
    ifDKStopOnMaxForce_2=False
    ifRobotStopOnMaxForce_2=False
    ifDKStopOnTimeDisMon=False
    ifRobotStopOnTimeDisMon=False
    ifNeedInit=True
    withGroup=False
    ftcSetGroup=[17]
    ignoreSensor=False


    FTC_setFTValueAll(isProgram, ftcProgram, onlyMonitor, graCalcIndex, ftEnabled, ftSet, dead_zone, disEndLimit,
                      timeEndLimit, ftEndLimit, disAng6D_EndLimit, ftcEndType, quickSetIndex, B, M, vel_limit,
                      cor_pos_limit, maxForce_1, ifDKStopOnMaxForce_1, ifRobotStopOnMaxForce_1, maxForce_2,
                      ifDKStopOnMaxForce_2, ifRobotStopOnMaxForce_2, ifDKStopOnTimeDisMon, ifRobotStopOnTimeDisMon,
                      ifNeedInit, withGroup, ftcSetGroup, ignoreSensor)

    index = 19  
    FTC_SetIndex(index)
    # 1 = enable DK assembly, 0 = disable DK assembly
    flag = 1
    FTC_SetDKAssemFlag(flag)

    time.sleep(10)
    print("Changed!\n")
    ftEnabled=[True,False,False,False,False,False]
    FTC_setFTsetAllRT(isProgram, ftcProgram, onlyMonitor, graCalcIndex, ftEnabled, ftSet, dead_zone, disEndLimit,
                      timeEndLimit, ftEndLimit, disAng6D_EndLimit, ftcEndType, quickSetIndex, B, M, vel_limit,
                      cor_pos_limit, maxForce_1, ifDKStopOnMaxForce_1, ifRobotStopOnMaxForce_1, maxForce_2,
                      ifDKStopOnMaxForce_2, ifRobotStopOnMaxForce_2, ifDKStopOnTimeDisMon, ifRobotStopOnTimeDisMon,
                      ifNeedInit, withGroup, ftcSetGroup, ignoreSensor)


    print("input 0 to stop force control")
    while True:
        try:
            user_input = input("input command (0 to disabled): ")
            if user_input.strip() == "0":
                FTC_SetDKAssemFlag(0)
                print("DK Assembly Flag is set to 0")
                break
            else:
                print("Invalid input, please enter 0 to stop force control")
        except KeyboardInterrupt:
            print("\n Detected Ctrl+C, stopping force control...")
            FTC_SetDKAssemFlag(0)
            print("DK Assembly Flag is set to 0")
            break
        except Exception as e:
            print(f"Input error: {e}")

    FTC_stop()
    print("FTC stopped")
if __name__ == "__main__":  
    main()  