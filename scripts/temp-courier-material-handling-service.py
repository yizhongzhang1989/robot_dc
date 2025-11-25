"""
FastAPI-based Courier Material Handling Service converted from SK Native plugin.
Based on list of functions Yizhong shared on 10/26 - 10/27, this is the temporary API for 10/30 integration testing.
"""

import logging
import os
import time

import requests
from dotenv import load_dotenv
from fastapi import FastAPI

load_dotenv()

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Configuration from environment variables
ACTION_HISTORY_FILE = "action_history.txt"
BASE_URL = os.getenv("BASE_URL", "http://localhost:8080")
MODE = os.getenv("MODE", "real")
API_HOST = os.getenv("API_HOST", "0.0.0.0")
COURIER_API_PORT = int(os.getenv("COURIER_API_PORT", "8001"))

logger.info(
    f"Configuration loaded - BASE_URL: {BASE_URL}, MODE: {MODE}, API_HOST: {API_HOST}, COURIER_API_PORT: {COURIER_API_PORT}"
)

# Initialize FastAPI app
app = FastAPI(
    title="Courier Material Handling Service",
    description="CourierMaterialHandlingService provides functionality to control the DC Courier to lift and transport a server.",
    version="1.0.0",
)


### HELPER FUNCTIONS ###
def _log_action(message: str):
    logger.info(message)
    with open(ACTION_HISTORY_FILE, "a") as f:
        f.write(message + "\n")


def _wait_for_program_completion(timeout: int = 200, check_interval: float = 1.0):
    """Wait for the program status to become 'Completed' before proceeding."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        status_response = _get_system_status_internal()
        if "Program: Completed" in status_response:
            msg = "Program status: Completed - proceeding to next step"
            _log_action(msg)
            return True
        elif "Program: Running" in status_response:
            msg = f"Program still running, waiting... ({int(time.time() - start_time)}s elapsed)"
            _log_action(msg)
            time.sleep(check_interval)
        else:
            msg = f"Program status unknown: {status_response}"
            _log_action(msg)
            time.sleep(check_interval)

    # Timeout reached
    timeout_msg = f"Timeout waiting for program completion after {timeout} seconds"
    _log_action(timeout_msg)
    raise Exception(timeout_msg)


def _run_program(program_name: str, block: bool = True) -> str:
    try:
        # Generate run_program command
        run_program_command = f"run_program {program_name} {str(block).lower()}"

        url_run_program = f"{BASE_URL}/api/robot_arm/cmd"
        params_run_program = {"command": run_program_command}
        headers = {"Content-Type": "application/json"}

        response = requests.post(
            url_run_program, json=params_run_program, headers=headers
        )
        time.sleep(2)

        if response.ok:
            msg = f"Run program command success: {program_name}"
            _log_action(msg)
            return msg
        else:
            msg = f"Failed to run program: {response.status_code}"
            _log_action(msg)
            return msg

    except Exception as e:
        msg = f"Error running program: {e}"
        _log_action(msg)
        return msg


#### TEMPORARY 10/30 INTEGRATION TEST ENDPOINTS ####


@app.post(
    "/MoveToServerRackForExtractions",
    summary="Move the courier robot to server rack for server extraction.",
    description="Move the courier robot to the server rack in the position necessary to extract the server (not to insert the server).",
    response_model=str,
    operation_id="moveToServerRackForExtraction",
)
async def move_to_server_rack_for_extraction():
    """Move the courier robot to the server rack for server extraction."""
    if MODE == "mock":
        time.sleep(10)
        msg = "Mock: Courier robot moved to server rack for server extraction successfully"
        _log_action(msg)
        return msg

    # TODO: Insert move_to_rack(dict) implementation here, called with 'pull' parameter
    msg = "Error: move_to_server_rack function not implemented"
    _log_action(msg)
    return msg

@app.post(
    "/MoveToServerRackForInsertion",
    summary="Move the courier robot to server rack for server insertion.",
    description="Move the courier robot to the server rack in the position necessary to insert the server (not to extract the server).",
    response_model=str,
    operation_id="moveToServerRackForInsertion",
)
async def move_to_server_rack_for_insertion():
    """Move the courier robot to the server rack for server insertion."""
    if MODE == "mock":
        time.sleep(10)
        msg = "Mock: Courier robot moved to server rack for server insertion successfully"
        _log_action(msg)
        return msg

    # TODO: Insert move_to_rack(dict) implementation here, called with 'push' parameter
    msg = "Error: move_to_server_rack function not implemented"
    _log_action(msg)
    return msg


@app.post(
    "/MoveAwayFromExtractionPositionByRack",
    summary="Move the courier robot away from the server extraction position by the server rack",
    description="Move the courier robot away from the rack, from where it was by the rack for server extraction.",
    response_model=str,
    operation_id="moveAwayFromExtractionPositionByRack",
)
async def move_away_from_extraction_position_by_rack():
    """Move the courier robot away from the server extraction position by the server rack."""
    if MODE == "mock":
        time.sleep(10)
        msg = "Mock: Courier robot moved away from server extraction position by rack successfully"
        _log_action(msg)
        return msg

    # TODO: Insert remove_from_rack(dict) implementation here, called with the pull parameter
    msg = "Error: move_away_from_rack function not implemented"
    _log_action(msg)
    return msg


@app.post(
    "/MoveAwayFromInsertionPositionByRack",
    summary="Move the courier robot away from the server insertion position by the server rack",
    description="Move the courier robot away from the rack, from where it was by the rack for server insertion.",
    response_model=str,
    operation_id="moveAwayFromInsertionPositionByRack",
)
async def move_away_from_insertion_position_by_rack():
    """Move the courier robot away from the server insertion position by the server rack."""
    if MODE == "mock":
        time.sleep(10)
        msg = "Mock: Courier robot moved away from server insertion position by rack successfully"
        _log_action(msg)
        return msg

    # TODO: Insert remove_from_rack(dict) implementation here, called with the push parameter
    msg = "Error: move_away_from_rack function not implemented"
    _log_action(msg)
    return msg


@app.post(
    "/AdjustPlatformHeight",
    summary="Adjust the height of the platform",
    description="Adjust the height of the platform to target height in millimeters using the x-frame and the linear actuators on the flow rails.",
    response_model=str,
    operation_id="adjustPlatformHeight",
)
async def adjust_platform_height(target_height: float):
    """Adjust the height of the platform.

    Args:
        target_height (float): The desired height for the platform in millimeters.
    """
    if MODE == "mock":
        time.sleep(10)
        msg = f"Mock: Platform moved to height {target_height} successfully"
        _log_action(msg)
        return msg

    # TODO: Insert move_x_lifter(dict) AND move_flow_rails(dict) implementation here (still discussing this in 2.4/2.1 Robot convergence Sync chat)
    msg = f"Error: adjust_platform_height function not implemented (target_height: {target_height})"
    _log_action(msg)
    return msg


@app.get(
    "/GetHeight",
    summary="Get current height of the platform",
    description="Get current height of the platform in millimeters.",
    response_model=float,
    operation_id="getHeight",
)
async def get_height() -> float:
    """Get current height of the platform in millimeters.
    Returns:
        float: Current height of the platform in millimeters.
    """
    if MODE == "mock":
        time.sleep(10)
        msg = "Mock: Current platform height: 100 mm"
        _log_action(msg)
        return 100.0

    # TODO: Insert get_height(dict) implementation here
    msg = "Error: get_height function not implemented"
    _log_action(msg)
    return {"error": msg}


@app.get(
    "/GetForce",
    summary="Get current force on the force sensors",
    description="Get current force on the force sensors on the platform in Newtons.",
    response_model=float,
    operation_id="getForce",
)
async def get_force() -> float:
    """Get current force on the force sensors on the platform.
    Returns:
        float: Current force on the force sensors in Newtons.
    """
    if MODE == "mock":
        time.sleep(10)
        return 25.5

    # TODO: Insert get_pressure(dict) implementation here
    msg = "Error: get_pressure function not implemented"
    _log_action(msg)
    return {"error": msg}


### COURIER ENDPOINTS THAT ALSO INVOLVE THE MANIPULATOR ROBOT, FOR NOW ###


@app.post(
    "/TransferServerOntoPlatform",
    summary="Transfer the server onto the courier platform",
    description="Transfer the server from the rack to the courier platform. The server must already be released from the rack, and courier at correct position by the rack for server extraction. During this process, the platform height will be adjusted as needed.",
    response_model=str,
    operation_id="transferServerOntoPlatform",
)
async def transfer_server_onto_platform():
    """Unlock the server knobs."""
    if MODE == "mock":
        time.sleep(10)
        msg = "Mock: Server transferred from rack to courier platform successfully"
        _log_action(msg)
        return msg

    # TODO: Insert move_server_rack_to_courier(dict) implementation here
    msg = "Error: transfer_server_onto_platform function not implemented"
    _log_action(msg)
    return msg


@app.post(
    "/TransferServerFromPlatform",
    summary="Transfer the server from the courier platform to the rack",
    description="Transfer the server from the courier platform to the rack. The courier must be in the correct position by the rack for server insertion. During this process, the platform height will be adjusted as needed.",
    response_model=str,
    operation_id="transferServerFromPlatform",
)
async def transfer_server_from_platform():
    """Transfer the server from the courier platform to the rack."""
    if MODE == "mock":
        time.sleep(10)
        msg = "Mock: Server transferred from courier platform to rack successfully"
        _log_action(msg)
        return msg

    # TODO: Insert move_server_courier_to_rack(dict) implementation here
    msg = "Error: transfer_server_onto_platform function not implemented"
    _log_action(msg)
    return msg


### STANDARD ENDPOINTS ###


# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "healthy"}


# Root endpoint
@app.get("/")
async def root():
    return {"message": "Courier Material Handling Service", "docs": "/docs"}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host=API_HOST, port=COURIER_API_PORT)