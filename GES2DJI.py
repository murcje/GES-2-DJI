import json
import xml.etree.ElementTree as ET
import zipfile
import os
import math
import argparse
import datetime
import tkinter as tk
from tkinter import filedialog, messagebox, ttk, simpledialog, scrolledtext # Added scrolledtext
import sys

# --- Try importing the map library ---
try:
    import tkintermapview
except ImportError:
    tkintermapview = None

# --- Constants and Core Conversion Logic ---

WPML_NAMESPACE = "http://www.dji.com/wpmz/1.0.2"
DEFAULT_SPEED_MPS = 12.0
DEFAULT_TRANSITIONAL_SPEED_MPS = 12.0
DEFAULT_WAYPOINT_SPEED_MPS = 12.0
DEFAULT_ALTITUDE_TYPE = 'WGS84'
DEFAULT_HEADING_MODE_WPML = 'smoothTransition'
DEFAULT_HEADING_PATH_MODE = 'followBadArc'
DEFAULT_TURN_MODE_INTERMEDIATE = 'toPointAndPassWithContinuityCurvature'
DEFAULT_TURN_MODE_LAST = 'toPointAndStopWithContinuityCurvature'
DEFAULT_TURN_DAMPING_DIST = 0.0
DEFAULT_FINISH_ACTION = 'goHome'
DEFAULT_RC_LOST_ACTION = 'goBack'
DRONE_ENUM_VALUE = 68
DRONE_SUB_ENUM_VALUE = 0

MIN_GIMBAL_PITCH = -90.0
MAX_GIMBAL_PITCH = 60.0
DEFAULT_GIMBAL_PITCH = 0.0

EARTH_RADIUS_METERS = 6371000

# --- Helper Functions ---

def thin_keyframes(keyframes_list, target_count, status_callback=None):
    """Thins keyframes using linear sampling."""
    original_count = len(keyframes_list)
    if not isinstance(target_count, int) or target_count <= 1 or target_count >= original_count:
        if status_callback and target_count is not None and target_count != 0 :
             status_callback(f"Invalid target count ({target_count}). No thinning applied.")
        elif status_callback:
             pass
        return keyframes_list

    if status_callback: status_callback(f"Thinning {original_count} waypoints to approximately {target_count}...")
    thinned_list = []
    indices_to_keep = set()
    for i in range(target_count):
        ideal_index = i * (original_count - 1) / (target_count - 1)
        index = round(ideal_index)
        index = max(0, min(original_count - 1, index))
        indices_to_keep.add(index)
    indices_to_keep.add(0)
    indices_to_keep.add(original_count - 1)
    sorted_indices = sorted(list(indices_to_keep))
    thinned_list = [keyframes_list[idx] for idx in sorted_indices]
    actual_thinned_count = len(thinned_list)
    if status_callback: status_callback(f"Thinning complete: {original_count} -> {actual_thinned_count} waypoints.")
    print(f"Waypoint thinning: Original={original_count}, Target={target_count}, Actual={actual_thinned_count}")
    return thinned_list

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculates the initial bearing (degrees) from point 1 to point 2."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLon = lon2 - lon1
    x = math.sin(dLon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    if bearing > 180:
        bearing -= 360
    return bearing

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculates the great-circle distance in meters using Haversine formula."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS_METERS * c
    return distance

def format_time(seconds):
    """Formats seconds into MM:SS string."""
    if seconds is None or seconds < 0:
        return "N/A"
    minutes = int(seconds // 60)
    remaining_seconds = int(seconds % 60)
    return f"{minutes:02d}:{remaining_seconds:02d}"

def find_keyframes(data):
    """Finds the list of keyframes within the loaded GES JSON data."""
    if isinstance(data, dict) and 'cameraFrames' in data:
        if isinstance(data['cameraFrames'], list):
            return data['cameraFrames']
        else:
            return None
    return None

def load_and_tag_keyframes(json_files_list, status_callback=None):
    """Loads keyframes and tags them with their origin file path."""
    all_tagged_keyframes = []
    if not json_files_list:
        raise ValueError("No JSON files selected.")

    for i, json_path in enumerate(json_files_list):
        if status_callback: status_callback(f"Loading file {i+1}/{len(json_files_list)}: {os.path.basename(json_path)}...")
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                ges_project_data = json.load(f)
            keyframes = find_keyframes(ges_project_data)
            if keyframes is None: raise ValueError(f"Could not find 'cameraFrames' list in {os.path.basename(json_path)}.")
            if not isinstance(keyframes, list): raise ValueError(f"'cameraFrames' in {os.path.basename(json_path)} is not a list.")
            if not keyframes:
                 print(f"Warning: No keyframes found in {os.path.basename(json_path)}. Skipping file.")
                 continue

            if not ('coordinate' in keyframes[0] and 'rotation' in keyframes[0]):
                 raise ValueError(f"First keyframe in {os.path.basename(json_path)} lacks 'coordinate' or 'rotation'.")

            # Tag each keyframe with its origin file
            for kf in keyframes:
                all_tagged_keyframes.append((kf, json_path)) # Store as tuple (keyframe_data, file_path)

            if status_callback: status_callback(f"Loaded {len(keyframes)} keyframes from {os.path.basename(json_path)}.")

        except FileNotFoundError: raise FileNotFoundError(f"Input file not found: {json_path}")
        except json.JSONDecodeError as e: raise json.JSONDecodeError(f"Invalid JSON format in file: {json_path}", e.doc, e.pos) # Pass error details
        except Exception as e: raise Exception(f"Error processing file {json_path}: {e}")

    if not all_tagged_keyframes:
        raise ValueError("No valid keyframes found in any selected JSON file.")

    if status_callback: status_callback(f"Total keyframes loaded: {len(all_tagged_keyframes)}")
    return all_tagged_keyframes


# --- Core Conversion Functions ---

def create_dji_wpml(tagged_thinned_keyframes, mission_settings, file_pois, status_callback=None):
    """Generates the XML content for waylines.wpml using tagged keyframes and per-file POIs."""
    if status_callback: status_callback("Generating WPML structure...")
    if not tagged_thinned_keyframes: raise ValueError("Thinned keyframes list is empty.")

    ET.register_namespace('wpml', WPML_NAMESPACE)
    ns = {'wpml': WPML_NAMESPACE}
    root = ET.Element("kml", xmlns="http://www.opengis.net/kml/2.2")
    doc = ET.SubElement(root, "Document")

    # Mission Config
    mission_config = ET.SubElement(doc, f"{{{WPML_NAMESPACE}}}missionConfig")
    ET.SubElement(mission_config, f"{{{WPML_NAMESPACE}}}flyToWaylineMode").text = "safely"
    ET.SubElement(mission_config, f"{{{WPML_NAMESPACE}}}finishAction").text = mission_settings.get('finishAction', DEFAULT_FINISH_ACTION)
    ET.SubElement(mission_config, f"{{{WPML_NAMESPACE}}}exitOnRCLost").text = "executeLostAction"
    ET.SubElement(mission_config, f"{{{WPML_NAMESPACE}}}executeRCLostAction").text = mission_settings.get('rcLostAction', DEFAULT_RC_LOST_ACTION)
    ET.SubElement(mission_config, f"{{{WPML_NAMESPACE}}}globalTransitionalSpeed").text = str(mission_settings.get('transitionalSpeed', DEFAULT_TRANSITIONAL_SPEED_MPS))
    drone_info = ET.SubElement(mission_config, f"{{{WPML_NAMESPACE}}}droneInfo")
    ET.SubElement(drone_info, f"{{{WPML_NAMESPACE}}}droneEnumValue").text = str(DRONE_ENUM_VALUE)
    ET.SubElement(drone_info, f"{{{WPML_NAMESPACE}}}droneSubEnumValue").text = str(DRONE_SUB_ENUM_VALUE)

    # Folder
    folder = ET.SubElement(doc, "Folder")
    ET.SubElement(folder, f"{{{WPML_NAMESPACE}}}templateId").text = "0"
    ET.SubElement(folder, f"{{{WPML_NAMESPACE}}}executeHeightMode").text = mission_settings.get('altitudeType', DEFAULT_ALTITUDE_TYPE)
    ET.SubElement(folder, f"{{{WPML_NAMESPACE}}}waylineId").text = "0"

    # --- Calculate Total Distance and Duration ---
    total_distance_meters = 0
    last_lat, last_lon = None, None
    calculated_duration_seconds = 0
    speed_mps = mission_settings.get('speed', DEFAULT_SPEED_MPS)

    for kf_data, _ in tagged_thinned_keyframes:
         try:
            lat = float(kf_data['coordinate']['latitude'])
            lon = float(kf_data['coordinate']['longitude'])
            if last_lat is not None:
                segment_distance = calculate_distance(last_lat, last_lon, lat, lon)
                total_distance_meters += segment_distance
            last_lat, last_lon = lat, lon
         except (KeyError, ValueError, TypeError): pass

    if speed_mps > 0: calculated_duration_seconds = total_distance_meters / speed_mps
    else: calculated_duration_seconds = 0

    ET.SubElement(folder, f"{{{WPML_NAMESPACE}}}distance").text = f"{total_distance_meters:.1f}"
    ET.SubElement(folder, f"{{{WPML_NAMESPACE}}}duration").text = f"{calculated_duration_seconds:.1f}"
    ET.SubElement(folder, f"{{{WPML_NAMESPACE}}}autoFlightSpeed").text = str(speed_mps)
    # -----------------------------------------

    # Altitude Reference (Based on the VERY FIRST point)
    first_kf_data, _ = tagged_thinned_keyframes[0]
    start_altitude = float(first_kf_data['coordinate']['altitude'])
    if status_callback: status_callback(f"Reference altitude (first point): {start_altitude:.2f}m")

    # Determine Fixed Pitch
    fixed_pitch_setting = mission_settings.get('fixed_pitch', None)
    target_pitch = DEFAULT_GIMBAL_PITCH
    if fixed_pitch_setting is not None:
        target_pitch = max(MIN_GIMBAL_PITCH, min(MAX_GIMBAL_PITCH, fixed_pitch_setting))
        if status_callback: status_callback(f"Using fixed gimbal pitch: {target_pitch:.1f}°")
    else:
        if status_callback: status_callback(f"Using default gimbal pitch: {target_pitch:.1f}° (Horizon)")

    # Heading Settings from GUI
    heading_mode_option = mission_settings.get('heading_mode', 'Follow Course')
    manual_heading = mission_settings.get('manual_heading')
    if status_callback: status_callback(f"Using Heading Mode: {heading_mode_option}")

    # Global Counters
    action_group_id_counter = 1
    action_id_counter = 1

    # Iterate through Tagged Keyframes
    num_keyframes = len(tagged_thinned_keyframes)
    for i, (kf, origin_file) in enumerate(tagged_thinned_keyframes):
        if status_callback and i % 10 == 0:
             status_callback(f"Processing waypoint {i+1}/{num_keyframes}...")

        placemark = ET.SubElement(folder, "Placemark")
        try:
            lat = float(kf['coordinate']['latitude'])
            lon = float(kf['coordinate']['longitude'])
            alt_amsl = float(kf['coordinate']['altitude'])
        except (KeyError, ValueError, TypeError) as e:
            if status_callback: status_callback(f"Warning: Skipping waypoint {i} from {os.path.basename(origin_file)} due to missing/invalid data: {e}")
            continue

        point = ET.SubElement(placemark, "Point")
        ET.SubElement(point, "coordinates").text = f"{lon:.8f},{lat:.8f}"
        ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}index").text = str(i)

        # Altitude
        execute_height = alt_amsl - start_altitude if mission_settings.get('altitudeType') != 'WGS84' else alt_amsl
        ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}executeHeight").text = f"{execute_height:.1f}"

        # Speed, Turn
        ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}waypointSpeed").text = str(mission_settings.get('waypointSpeed', DEFAULT_WAYPOINT_SPEED_MPS))
        turn_param = ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}waypointTurnParam")
        turn_mode = DEFAULT_TURN_MODE_LAST if i == num_keyframes - 1 else DEFAULT_TURN_MODE_INTERMEDIATE
        ET.SubElement(turn_param, f"{{{WPML_NAMESPACE}}}waypointTurnMode").text = turn_mode
        ET.SubElement(turn_param, f"{{{WPML_NAMESPACE}}}waypointTurnDampingDist").text = str(mission_settings.get('turnDamping', DEFAULT_TURN_DAMPING_DIST))
        ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}useStraightLine").text = "0"

        # --- Heading Logic with Per-File POI ---
        heading_param = ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}waypointHeadingParam")
        wp_heading_mode = 'followWayline'
        wp_heading_angle = 0.0
        wp_heading_angle_enable = '0'

        if heading_mode_option == "Point Towards File's POI":
            poi_coords = file_pois.get(origin_file)
            if poi_coords:
                center_lat, center_lon = poi_coords
                wp_heading_mode = DEFAULT_HEADING_MODE_WPML # smoothTransition
                wp_heading_angle = calculate_bearing(lat, lon, center_lat, center_lon)
                wp_heading_angle_enable = '1' if (i == 0 or i == num_keyframes - 1) else '0'
            else:
                print(f"Warning: POI not set for file '{os.path.basename(origin_file)}' used by waypoint {i}. Defaulting heading to 'followWayline'.")
                wp_heading_mode = 'followWayline'
                wp_heading_angle_enable = '0'
        elif heading_mode_option == 'Manual Fixed Heading':
            wp_heading_mode = DEFAULT_HEADING_MODE_WPML # smoothTransition
            wp_heading_angle = manual_heading
            wp_heading_angle_enable = '1' if (i == 0 or i == num_keyframes - 1) else '0'
        # Else (Follow Course): Keep defaults

        ET.SubElement(heading_param, f"{{{WPML_NAMESPACE}}}waypointHeadingMode").text = wp_heading_mode
        ET.SubElement(heading_param, f"{{{WPML_NAMESPACE}}}waypointHeadingAngle").text = f"{wp_heading_angle:.1f}"
        ET.SubElement(heading_param, f"{{{WPML_NAMESPACE}}}waypointPoiPoint").text = "0.000000,0.000000,0.000000"
        ET.SubElement(heading_param, f"{{{WPML_NAMESPACE}}}waypointHeadingAngleEnable").text = wp_heading_angle_enable
        ET.SubElement(heading_param, f"{{{WPML_NAMESPACE}}}waypointHeadingPathMode").text = DEFAULT_HEADING_PATH_MODE


        # --- Action Group 1: Gimbal Rotate on Reach Point ---
        action_group_reach = ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}actionGroup")
        current_action_group_id = action_group_id_counter; action_group_id_counter += 1
        ET.SubElement(action_group_reach, f"{{{WPML_NAMESPACE}}}actionGroupId").text = str(current_action_group_id)
        ET.SubElement(action_group_reach, f"{{{WPML_NAMESPACE}}}actionGroupStartIndex").text = str(i)
        ET.SubElement(action_group_reach, f"{{{WPML_NAMESPACE}}}actionGroupEndIndex").text = str(i)
        ET.SubElement(action_group_reach, f"{{{WPML_NAMESPACE}}}actionGroupMode").text = "parallel"
        action_trigger_reach = ET.SubElement(action_group_reach, f"{{{WPML_NAMESPACE}}}actionTrigger")
        ET.SubElement(action_trigger_reach, f"{{{WPML_NAMESPACE}}}actionTriggerType").text = "reachPoint"
        action_reach = ET.SubElement(action_group_reach, f"{{{WPML_NAMESPACE}}}action")
        current_action_id = action_id_counter; action_id_counter += 1
        ET.SubElement(action_reach, f"{{{WPML_NAMESPACE}}}actionId").text = str(current_action_id)
        ET.SubElement(action_reach, f"{{{WPML_NAMESPACE}}}actionActuatorFunc").text = "gimbalRotate"
        actuator_param_reach = ET.SubElement(action_reach, f"{{{WPML_NAMESPACE}}}actionActuatorFuncParam")
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalHeadingYawBase").text = "aircraft"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalRotateMode").text = "absoluteAngle"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalPitchRotateEnable").text = "1"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalPitchRotateAngle").text = f"{target_pitch:.1f}"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalRollRotateEnable").text = "0"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalRollRotateAngle").text = "0"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalYawRotateEnable").text = "0"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalYawRotateAngle").text = "0"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalRotateTimeEnable").text = "0"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}gimbalRotateTime").text = "0"
        ET.SubElement(actuator_param_reach, f"{{{WPML_NAMESPACE}}}payloadPositionIndex").text = "0"

        # --- Action Group 2: Gimbal Evenly Rotate Between Points ---
        if i < num_keyframes - 1:
            action_group_between = ET.SubElement(placemark, f"{{{WPML_NAMESPACE}}}actionGroup")
            current_action_group_id = action_group_id_counter; action_group_id_counter += 1
            ET.SubElement(action_group_between, f"{{{WPML_NAMESPACE}}}actionGroupId").text = str(current_action_group_id)
            ET.SubElement(action_group_between, f"{{{WPML_NAMESPACE}}}actionGroupStartIndex").text = str(i)
            ET.SubElement(action_group_between, f"{{{WPML_NAMESPACE}}}actionGroupEndIndex").text = str(i + 1)
            ET.SubElement(action_group_between, f"{{{WPML_NAMESPACE}}}actionGroupMode").text = "parallel"
            action_trigger_between = ET.SubElement(action_group_between, f"{{{WPML_NAMESPACE}}}actionTrigger")
            trigger_type = "reachPoint" if i == 0 else "betweenAdjacentPoints"
            ET.SubElement(action_trigger_between, f"{{{WPML_NAMESPACE}}}actionTriggerType").text = trigger_type
            action_between = ET.SubElement(action_group_between, f"{{{WPML_NAMESPACE}}}action")
            current_action_id = action_id_counter; action_id_counter += 1
            ET.SubElement(action_between, f"{{{WPML_NAMESPACE}}}actionId").text = str(current_action_id)
            ET.SubElement(action_between, f"{{{WPML_NAMESPACE}}}actionActuatorFunc").text = "gimbalEvenlyRotate"
            actuator_param_between = ET.SubElement(action_between, f"{{{WPML_NAMESPACE}}}actionActuatorFuncParam")
            ET.SubElement(actuator_param_between, f"{{{WPML_NAMESPACE}}}gimbalPitchRotateAngle").text = f"{target_pitch:.1f}"
            ET.SubElement(actuator_param_between, f"{{{WPML_NAMESPACE}}}payloadPositionIndex").text = "0"

    try:
        ET.indent(root, space="  ")
    except AttributeError:
        pass

    xml_string = ET.tostring(root, encoding='unicode', method='xml')
    if status_callback: status_callback("WPML structure generated.")
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + xml_string

# create_template_kml remains the same
def create_template_kml(ref_kml_content=None, status_callback=None):
    """Creates the template.kml content."""
    if ref_kml_content:
        if "<wpml:missionConfig>" in ref_kml_content and "<wpml:droneInfo>" in ref_kml_content:
             if status_callback: status_callback("Using provided reference template.kml.")
             return ref_kml_content
        else:
             if status_callback: status_callback("Warning: Provided reference KML incomplete. Generating default.")

    if status_callback: status_callback("Generating default template.kml.")
    try:
        timestamp_ms = str(int(datetime.datetime.now(datetime.timezone.utc).timestamp() * 1000))
    except Exception:
        timestamp_ms = "0"

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="{WPML_NAMESPACE}">
  <Document>
    <wpml:author>GES_Converter_GUI</wpml:author>
    <wpml:createTime>{timestamp_ms}</wpml:createTime>
    <wpml:updateTime>{timestamp_ms}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>{DEFAULT_FINISH_ACTION}</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>{DEFAULT_RC_LOST_ACTION}</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>{DEFAULT_TRANSITIONAL_SPEED_MPS:.1f}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>{DRONE_ENUM_VALUE}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>{DRONE_SUB_ENUM_VALUE}</wpml:droneSubEnumValue>
      </wpml:droneInfo>
    </wpml:missionConfig>
    <Folder>
        <Placemark>
            <Point><coordinates>0,0</coordinates></Point>
        </Placemark>
    </Folder>
  </Document>
</kml>"""

# ges_json_to_dji_kmz (Handles merging now)
def ges_json_to_dji_kmz(json_files_list, kmz_path, ref_kml_path=None, mission_settings=None, file_pois=None, status_callback=None):
    """Converts one or more GES JSON files to a DJI Mini 4 Pro KMZ file."""
    if mission_settings is None: mission_settings = {}
    if file_pois is None: file_pois = {}

    try:
        # 1. Load and Merge Keyframes from potentially multiple files
        all_tagged_keyframes = load_and_tag_keyframes(json_files_list, status_callback)
        original_kf_count = len(all_tagged_keyframes) # Count after merging

        # --- Waypoint Thinning Step (on merged list) ---
        desired_waypoints = mission_settings.get('desired_waypoints', None)
        thinned_tagged_keyframes = thin_keyframes(all_tagged_keyframes, desired_waypoints, status_callback)
        # -----------------------------

        # 3. Generate WPML content (using the potentially thinned list and POI dict)
        wpml_content = create_dji_wpml(thinned_tagged_keyframes, mission_settings, file_pois, status_callback)

        # 4. Get template.kml content
        ref_kml_content = None
        if ref_kml_path and os.path.exists(ref_kml_path):
            if status_callback: status_callback(f"Loading reference KML: {os.path.basename(ref_kml_path)}")
            try:
                with open(ref_kml_path, 'r', encoding='utf-8') as f:
                    ref_kml_content = f.read()
            except Exception as e:
                 if status_callback: status_callback(f"Warning: Could not read reference KML: {e}")
                 ref_kml_content = None
        else:
             if ref_kml_path:
                  if status_callback: status_callback(f"Warning: Reference KML not found: {os.path.basename(ref_kml_path)}")

        kml_content = create_template_kml(ref_kml_content, status_callback)

        # 5. Create KMZ
        if status_callback: status_callback(f"Creating KMZ: {os.path.basename(kmz_path)}")
        output_dir = os.path.dirname(kmz_path)
        if output_dir and not os.path.exists(output_dir):
             os.makedirs(output_dir)

        with zipfile.ZipFile(kmz_path, 'w', zipfile.ZIP_DEFLATED) as zf:
            zf.writestr('wpmz/waylines.wpml', wpml_content.encode('utf-8'))
            zf.writestr('wpmz/template.kml', kml_content.encode('utf-8'))

        if status_callback: status_callback(f"Successfully created KMZ: {os.path.basename(kmz_path)}")
        return True

    except Exception as e:
        if status_callback: status_callback(f"Error: {e}")
        import traceback
        print(f"An unexpected error occurred: {e}")
        print("Traceback:")
        traceback.print_exc()
        return False


# --- GUI Application Class ---
class ConverterApp:
    def __init__(self, master):
        self.master = master
        master.title("GES JSON to DJI KMZ Converter")
        master.geometry("650x700") # Adjusted height

        # Style
        style = ttk.Style()
        style.theme_use('clam')

        # --- Variables ---
        self.json_files_list = [] # Store paths of selected JSON files
        self.file_pois = {}       # Dictionary to store {filepath: (lat, lon)}
        self.kmz_file_var = tk.StringVar()
        self.ref_kml_file_var = tk.StringVar()
        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED_MPS)
        self.altitude_type_var = tk.StringVar(value=DEFAULT_ALTITUDE_TYPE)
        self.finish_action_var = tk.StringVar(value=DEFAULT_FINISH_ACTION)
        self.rc_lost_var = tk.StringVar(value=DEFAULT_RC_LOST_ACTION)
        self.turn_damping_var = tk.DoubleVar(value=DEFAULT_TURN_DAMPING_DIST)
        self.desired_waypoints_var = tk.StringVar()
        self.fixed_pitch_var = tk.StringVar()
        # Heading Variables
        self.heading_mode_var = tk.StringVar(value="Follow Course")
        self.selected_file_poi_lat_var = tk.StringVar() # For the POI entry fields
        self.selected_file_poi_lon_var = tk.StringVar()
        self.manual_heading_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Ready.")
        # Preview info variables
        self.preview_info_var = tk.StringVar(value="")

        # Store map widget and markers if preview window is open
        self.map_preview_window = None
        self.map_widget = None
        self.map_markers = []
        self.map_path = None
        self.poi_marker = None

        # --- Layout Frames ---
        file_frame = ttk.LabelFrame(master, text="Input GES JSON File(s)", padding=(10, 5))
        file_frame.pack(padx=10, pady=5, fill=tk.X)

        output_frame = ttk.LabelFrame(master, text="Output & Reference Files", padding=(10, 5))
        output_frame.pack(padx=10, pady=5, fill=tk.X)

        settings_frame = ttk.LabelFrame(master, text="Mission Settings", padding=(10, 5))
        settings_frame.pack(padx=10, pady=5, fill=tk.X)

        general_settings_frame = ttk.Frame(settings_frame, padding=(0, 0))
        general_settings_frame.pack(fill=tk.X, pady=(0,5))

        heading_frame = ttk.LabelFrame(settings_frame, text="Heading Control", padding=(10, 5))
        heading_frame.pack(padx=5, pady=(5,5), fill=tk.X)

        action_frame = ttk.Frame(master, padding=(10, 10))
        action_frame.pack(padx=10, pady=5, fill=tk.X)

        status_frame = ttk.Frame(master, padding=(10, 5))
        status_frame.pack(padx=10, pady=5, fill=tk.X, side=tk.BOTTOM)


        # --- File Selection Widgets (Multi-file) ---
        listbox_frame = ttk.Frame(file_frame)
        listbox_frame.pack(pady=5, fill=tk.X)

        self.file_listbox = tk.Listbox(listbox_frame, height=4, width=70, exportselection=False)
        self.file_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.file_listbox.bind("<<ListboxSelect>>", self.on_file_select) # Bind selection change

        listbox_scrollbar = ttk.Scrollbar(listbox_frame, orient=tk.VERTICAL, command=self.file_listbox.yview)
        listbox_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.file_listbox.config(yscrollcommand=listbox_scrollbar.set)

        file_button_frame = ttk.Frame(file_frame)
        file_button_frame.pack(pady=2)
        ttk.Button(file_button_frame, text="Add File(s)...", command=self.add_files).pack(side=tk.LEFT, padx=5)
        ttk.Button(file_button_frame, text="Remove Selected", command=self.remove_selected_file).pack(side=tk.LEFT, padx=5)

        # --- Per-File POI Setting ---
        poi_frame = ttk.LabelFrame(file_frame, text="Set POI for Selected File (Required for 'Point Towards File's POI' Heading)", padding=(10, 5))
        poi_frame.pack(pady=5, fill=tk.X)

        ttk.Label(poi_frame, text="POI Lat:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.poi_lat_entry = ttk.Entry(poi_frame, textvariable=self.selected_file_poi_lat_var, width=15)
        self.poi_lat_entry.grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)

        ttk.Label(poi_frame, text="POI Lon:").grid(row=0, column=2, padx=5, pady=2, sticky=tk.W)
        self.poi_lon_entry = ttk.Entry(poi_frame, textvariable=self.selected_file_poi_lon_var, width=15)
        self.poi_lon_entry.grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)

        self.set_poi_button = ttk.Button(poi_frame, text="Set POI for Selected", command=self.set_poi_for_selected, state=tk.DISABLED)
        self.set_poi_button.grid(row=0, column=4, padx=10, pady=2)


        # --- Output/Reference File Widgets ---
        ttk.Label(output_frame, text="Output KMZ File:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(output_frame, textvariable=self.kmz_file_var, width=50).grid(row=0, column=1, padx=5, pady=5, sticky=tk.EW)
        ttk.Button(output_frame, text="Browse...", command=self.browse_kmz).grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(output_frame, text="Ref Template KML:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(output_frame, textvariable=self.ref_kml_file_var, width=50).grid(row=1, column=1, padx=5, pady=5, sticky=tk.EW)
        ttk.Button(output_frame, text="Browse...", command=self.browse_ref_kml).grid(row=1, column=2, padx=5, pady=5)

        output_frame.columnconfigure(1, weight=1)


        # --- General Settings Widgets ---
        # (Same layout as before, just placed in general_settings_frame)
        ttk.Label(general_settings_frame, text="Speed (m/s):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(general_settings_frame, textvariable=self.speed_var, width=10).grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        ttk.Label(general_settings_frame, text="Altitude Type:").grid(row=0, column=2, padx=15, pady=5, sticky=tk.W)
        ttk.OptionMenu(general_settings_frame, self.altitude_type_var, DEFAULT_ALTITUDE_TYPE, 'relativeToStartPoint', 'WGS84').grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        ttk.Label(general_settings_frame, text="Finish Action:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.OptionMenu(general_settings_frame, self.finish_action_var, DEFAULT_FINISH_ACTION, 'goHome', 'autoLand', 'hover').grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        ttk.Label(general_settings_frame, text="RC Lost Action:").grid(row=1, column=2, padx=15, pady=5, sticky=tk.W)
        ttk.OptionMenu(general_settings_frame, self.rc_lost_var, DEFAULT_RC_LOST_ACTION, 'goBack', 'hover', 'autoLand').grid(row=1, column=3, padx=5, pady=5, sticky=tk.W)
        ttk.Label(general_settings_frame, text="Turn Damping:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(general_settings_frame, textvariable=self.turn_damping_var, width=10).grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)
        ttk.Label(general_settings_frame, text="Desired Waypoints (Optional):").grid(row=2, column=2, padx=15, pady=5, sticky=tk.W)
        ttk.Entry(general_settings_frame, textvariable=self.desired_waypoints_var, width=10).grid(row=2, column=3, padx=5, pady=5, sticky=tk.W)
        ttk.Label(general_settings_frame, text="Fixed Gimbal Pitch (Optional):").grid(row=3, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(general_settings_frame, textvariable=self.fixed_pitch_var, width=10).grid(row=3, column=1, padx=5, pady=5, sticky=tk.W)
        ttk.Label(general_settings_frame, text=f"° (e.g., -90 to {MAX_GIMBAL_PITCH:.0f})").grid(row=3, column=2, padx=0, pady=5, sticky=tk.W)


        # --- Heading Control Widgets ---
        # Modified heading options
        heading_options = ["Follow Course", "Point Towards File's POI", "Manual Fixed Heading"]
        ttk.Label(heading_frame, text="Heading Mode:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        heading_menu = ttk.OptionMenu(heading_frame, self.heading_mode_var, heading_options[0], *heading_options, command=self.toggle_heading_fields)
        heading_menu.grid(row=0, column=1, columnspan=3, padx=5, pady=5, sticky=tk.W+tk.E)

        # Manual Heading Field (Now in heading_frame)
        self.manual_heading_label = ttk.Label(heading_frame, text="Manual Heading Angle:")
        self.manual_heading_entry = ttk.Entry(heading_frame, textvariable=self.manual_heading_var, width=10)
        self.manual_heading_unit_label = ttk.Label(heading_frame, text="° (-180 to 180)")

        self.manual_heading_label.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.manual_heading_entry.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        self.manual_heading_unit_label.grid(row=1, column=2, padx=0, pady=5, sticky=tk.W)

        # Initially disable/enable fields based on default heading mode
        self.toggle_heading_fields(self.heading_mode_var.get())


        # --- Action Buttons Frame ---
        buttons_subframe = ttk.Frame(action_frame)
        buttons_subframe.pack(pady=10)

        # Preview Button (Now previews selected file)
        self.preview_button = ttk.Button(buttons_subframe, text="Preview Selected File & Set POI", command=self.preview_path, state=tk.DISABLED)
        self.preview_button.pack(side=tk.LEFT, padx=5)
        if tkintermapview is None:
            self.preview_button.config(state=tk.DISABLED)
            self.status_var.set("Error: tkintermapview not found. Install it (`pip install tkintermapview pillow`).")

        # Convert Button
        self.convert_button = ttk.Button(buttons_subframe, text="Convert & Merge to KMZ", command=self.convert)
        self.convert_button.pack(side=tk.LEFT, padx=5)

        # --- Status Bar ---
        ttk.Label(status_frame, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(fill=tk.X)

    def toggle_heading_fields(self, selected_mode):
        """Enable/disable specific heading fields based on selected mode."""
        # Disable all first
        # self.center_lat_label.config(state=tk.DISABLED) # Removed global center fields
        # self.center_lat_entry.config(state=tk.DISABLED)
        # self.center_lon_label.config(state=tk.DISABLED)
        # self.center_lon_entry.config(state=tk.DISABLED)
        self.manual_heading_label.config(state=tk.DISABLED)
        self.manual_heading_entry.config(state=tk.DISABLED)
        self.manual_heading_unit_label.config(state=tk.DISABLED)

        # Enable based on selection
        if selected_mode == "Manual Fixed Heading":
            self.manual_heading_label.config(state=tk.NORMAL)
            self.manual_heading_entry.config(state=tk.NORMAL)
            self.manual_heading_unit_label.config(state=tk.NORMAL)
        # "Point Towards File's POI" fields are managed separately by file selection

    # --- File Management Functions ---
    def add_files(self):
        """Adds one or more JSON files to the listbox."""
        filenames_tuple = filedialog.askopenfilenames(
            title="Select GES JSON File(s) to Add",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*"))
        )
        added_count = 0
        if filenames_tuple:
            for fname in filenames_tuple:
                if fname not in self.json_files_list: # Avoid duplicates
                    self.json_files_list.append(fname)
                    self.file_listbox.insert(tk.END, os.path.basename(fname))
                    added_count += 1
            if added_count > 0:
                self.update_status(f"Added {added_count} file(s). Total: {len(self.json_files_list)}")
                # Auto-suggest output filename if not set
                if not self.kmz_file_var.get() and self.json_files_list:
                    base, _ = os.path.splitext(self.json_files_list[0])
                    self.kmz_file_var.set(base + "_merged.kmz")
            else:
                 self.update_status("No new files added (already in list).")


    def remove_selected_file(self):
        """Removes the selected file from the listbox and internal list."""
        selected_indices = self.file_listbox.curselection()
        if not selected_indices:
            messagebox.showwarning("Warning", "Please select a file from the list to remove.")
            return

        # Remove from listbox (iterate backwards to avoid index issues)
        for index in reversed(selected_indices):
            filepath = self.json_files_list.pop(index) # Remove from internal list
            self.file_listbox.delete(index)
            # Remove associated POI if it exists
            if filepath in self.file_pois:
                del self.file_pois[filepath]
                print(f"Removed POI for {os.path.basename(filepath)}")

        self.update_status(f"Removed file(s). Remaining: {len(self.json_files_list)}")
        # Clear POI fields if nothing is selected anymore
        if not self.file_listbox.curselection():
             self.selected_file_poi_lat_var.set("")
             self.selected_file_poi_lon_var.set("")
             self.set_poi_button.config(state=tk.DISABLED)
             self.preview_button.config(state=tk.DISABLED)


    def on_file_select(self, event):
        """Updates POI fields when a file is selected in the listbox."""
        selected_indices = self.file_listbox.curselection()
        if not selected_indices:
            self.selected_file_poi_lat_var.set("")
            self.selected_file_poi_lon_var.set("")
            self.set_poi_button.config(state=tk.DISABLED)
            self.preview_button.config(state=tk.DISABLED)
            return

        selected_index = selected_indices[0] # Only handle single selection for POI
        selected_filepath = self.json_files_list[selected_index]

        # Enable buttons for selected file
        self.set_poi_button.config(state=tk.NORMAL)
        if tkintermapview is not None:
            self.preview_button.config(state=tk.NORMAL)

        # Load and display existing POI for this file, if any
        poi_coords = self.file_pois.get(selected_filepath)
        if poi_coords:
            self.selected_file_poi_lat_var.set(f"{poi_coords[0]:.8f}")
            self.selected_file_poi_lon_var.set(f"{poi_coords[1]:.8f}")
            self.update_status(f"Selected: {os.path.basename(selected_filepath)} (POI Loaded)")
        else:
            self.selected_file_poi_lat_var.set("")
            self.selected_file_poi_lon_var.set("")
            self.update_status(f"Selected: {os.path.basename(selected_filepath)} (No POI Set)")


    def set_poi_for_selected(self):
        """Saves the entered Lat/Lon as the POI for the selected file."""
        selected_indices = self.file_listbox.curselection()
        if not selected_indices:
            messagebox.showwarning("Warning", "Please select a file from the list first.")
            return
        selected_index = selected_indices[0]
        selected_filepath = self.json_files_list[selected_index]

        try:
            lat_str = self.selected_file_poi_lat_var.get()
            lon_str = self.selected_file_poi_lon_var.get()
            if not lat_str or not lon_str:
                # Clear POI if fields are empty
                if selected_filepath in self.file_pois:
                    del self.file_pois[selected_filepath]
                messagebox.showinfo("POI Cleared", f"POI cleared for {os.path.basename(selected_filepath)}")
                self.update_status(f"POI cleared for {os.path.basename(selected_filepath)}")
                return

            lat = float(lat_str)
            lon = float(lon_str)
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError("Invalid Latitude (-90 to 90) or Longitude (-180 to 180).")

            # Store the POI
            self.file_pois[selected_filepath] = (lat, lon)
            print(f"Stored POI for {os.path.basename(selected_filepath)}: {self.file_pois[selected_filepath]}")
            self.update_status(f"POI set for {os.path.basename(selected_filepath)}")
            messagebox.showinfo("POI Set", f"POI coordinates saved for:\n{os.path.basename(selected_filepath)}")

        except ValueError as e:
            messagebox.showerror("Error", f"Invalid POI coordinates: {e}")
        except tk.TclError:
            messagebox.showerror("Error", "Invalid value entered for POI Latitude or Longitude.")


    # --- File Browsing Functions (Output/Ref) ---
    def browse_kmz(self):
        filename = filedialog.asksaveasfilename(
            title="Save KMZ File As",
            defaultextension=".kmz",
            filetypes=(("KMZ files", "*.kmz"), ("All files", "*.*"))
        )
        if filename:
            self.kmz_file_var.set(filename)
            self.update_status(f"Output KMZ set: {os.path.basename(filename)}")

    def browse_ref_kml(self):
        filename = filedialog.askopenfilename(
            title="Select Reference Template KML File (Optional)",
            filetypes=(("KML/Text files", "*.kml *.txt"), ("All files", "*.*"))
        )
        if filename:
            self.ref_kml_file_var.set(filename)
            self.update_status(f"Selected Ref KML: {os.path.basename(filename)}")
        else:
            self.ref_kml_file_var.set("")
            self.update_status("Reference KML cleared.")

    def update_status(self, message):
        self.status_var.set(message)
        self.master.update_idletasks()

    def map_clicked(self, coords):
        """Callback function when the map preview is clicked."""
        # Only update POI fields if a file is selected in the main list
        selected_indices = self.file_listbox.curselection()
        if not selected_indices:
            self.update_status("Select a file in the main window first to set its POI.")
            return

        lat, lon = coords
        print(f"Map clicked at: Lat={lat:.6f}, Lon={lon:.6f}")
        self.update_status(f"Map Clicked: Lat={lat:.6f}, Lon={lon:.6f}. Use 'Set POI' button to save.")

        # Update the POI entry fields (ready to be saved by button)
        self.selected_file_poi_lat_var.set(f"{lat:.8f}")
        self.selected_file_poi_lon_var.set(f"{lon:.8f}")

        # Add/update a marker on the map for the potential POI
        if self.map_widget:
             if self.poi_marker:
                 self.poi_marker.set_position(lat, lon)
             else:
                 self.poi_marker = self.map_widget.set_marker(lat, lon, text="POI", marker_color_circle="red", marker_color_outside="red")

    def preview_path(self):
        """Loads JSON for the SELECTED file, thins, calculates info, and displays."""
        if tkintermapview is None:
            messagebox.showerror("Error", "Map preview requires 'tkintermapview'.\nPlease install it using:\n pip install tkintermapview pillow")
            return

        # Get selected file path
        selected_indices = self.file_listbox.curselection()
        if not selected_indices:
            messagebox.showerror("Error", "Please select a JSON file from the list to preview.")
            return
        selected_index = selected_indices[0]
        json_path = self.json_files_list[selected_index]

        # Get desired waypoints and speed
        desired_waypoints_val = None
        speed_mps_val = DEFAULT_SPEED_MPS
        try:
            desired_waypoints_str = self.desired_waypoints_var.get()
            if desired_waypoints_str:
                desired_waypoints_val = int(desired_waypoints_str)
                if desired_waypoints_val <= 0: raise ValueError("Desired Waypoints must be positive.")
            speed_mps_val = self.speed_var.get()
            if speed_mps_val <= 0: speed_mps_val = 0 # Avoid division error
        except (ValueError, tk.TclError) as e:
             messagebox.showerror("Error", f"Invalid numeric setting for preview:\n{e}")
             return

        self.update_status(f"Loading preview for {os.path.basename(json_path)}...")
        try:
            # Load keyframes ONLY for the selected file
            with open(json_path, 'r', encoding='utf-8') as f:
                ges_project_data = json.load(f)
            camera_keyframes = find_keyframes(ges_project_data)
            if camera_keyframes is None: raise ValueError("Could not find 'cameraFrames'.")
            if not camera_keyframes: raise ValueError("No keyframes found.")

            # Perform thinning on this file's keyframes
            thinned_keyframes = thin_keyframes(camera_keyframes, desired_waypoints_val, self.update_status)
            if not thinned_keyframes: raise ValueError("No waypoints remaining after thinning.")

            # --- Calculate Distance and Time for THIS file ---
            total_distance_meters = 0
            last_lat, last_lon = None, None
            for i, kf in enumerate(thinned_keyframes):
                try:
                    lat = float(kf['coordinate']['latitude'])
                    lon = float(kf['coordinate']['longitude'])
                    if last_lat is not None:
                        segment_distance = calculate_distance(last_lat, last_lon, lat, lon)
                        total_distance_meters += segment_distance
                    last_lat, last_lon = lat, lon
                except (KeyError, ValueError, TypeError): continue

            estimated_time_seconds = None
            if speed_mps_val > 0: estimated_time_seconds = total_distance_meters / speed_mps_val
            distance_km = total_distance_meters / 1000.0
            time_str = format_time(estimated_time_seconds)
            info_text = f"Selected File: {distance_km:.3f} km | Est. Time: {time_str} (at {speed_mps_val:.1f} m/s)"
            # ------------------------------------

            self.update_status("Preparing map preview...")

            # --- Create or Update Map Window ---
            # Increased height slightly for the text widget
            preview_window_height = 750
            map_height = 400
            text_height = 200

            if self.map_preview_window is None or not self.map_preview_window.winfo_exists():
                self.map_preview_window = tk.Toplevel(self.master)
                self.map_preview_window.title(f"Preview & POI: {os.path.basename(json_path)}")
                self.map_preview_window.geometry(f"800x{preview_window_height}") # Adjusted size

                # Top frame for instructions and info
                top_frame = ttk.Frame(self.map_preview_window)
                top_frame.pack(pady=5, fill=tk.X, padx=10)
                ttk.Label(top_frame, text="Click map to set POI for this file.").pack(side=tk.LEFT)
                self.preview_info_label = ttk.Label(top_frame, textvariable=self.preview_info_var)
                self.preview_info_label.pack(side=tk.RIGHT)

                # Map widget
                self.map_widget = tkintermapview.TkinterMapView(self.map_preview_window, width=800, height=map_height, corner_radius=0)
                self.map_widget.pack(fill=tk.X)
                self.map_widget.add_left_click_map_command(self.map_clicked)

                # Text widget for JSON data
                ttk.Label(self.map_preview_window, text="Camera Frames Data:").pack(pady=(5,0))
                self.json_text_widget = scrolledtext.ScrolledText(self.map_preview_window, height=int(text_height/15), wrap=tk.WORD, state=tk.DISABLED) # Approx height based on font size
                self.json_text_widget.pack(pady=5, padx=10, fill=tk.BOTH, expand=True)

                self.map_markers = []
                self.map_path = None
                self.poi_marker = None # Reset POI marker when window is created
            else:
                self.map_preview_window.lift()
                self.map_preview_window.title(f"Preview & POI: {os.path.basename(json_path)}") # Update title
                if self.map_widget:
                    for marker in self.map_markers: marker.delete()
                    self.map_markers = []
                    if self.map_path: self.map_path.delete(); self.map_path = None
                    # Don't delete POI marker here, allow it to persist if user previews another file

            # --- Update Preview Info Label ---
            self.preview_info_var.set(info_text)
            # --------------------------------

            # --- Display JSON Data ---
            try:
                 # Extract only cameraFrames for display
                 frames_to_display = ges_project_data.get('cameraFrames', [])
                 formatted_json = json.dumps(frames_to_display, indent=2)
                 self.json_text_widget.config(state=tk.NORMAL) # Enable writing
                 self.json_text_widget.delete('1.0', tk.END)    # Clear previous content
                 self.json_text_widget.insert('1.0', formatted_json)
                 self.json_text_widget.config(state=tk.DISABLED) # Disable editing
            except Exception as json_e:
                 print(f"Error formatting/displaying JSON: {json_e}")
                 self.json_text_widget.config(state=tk.NORMAL)
                 self.json_text_widget.delete('1.0', tk.END)
                 self.json_text_widget.insert('1.0', f"Error displaying JSON data:\n{json_e}")
                 self.json_text_widget.config(state=tk.DISABLED)
            # -------------------------


            # --- Plot Waypoints ---
            marker_positions = []
            path_positions = []
            min_lat, max_lat = 90.0, -90.0
            min_lon, max_lon = 180.0, -180.0

            for i, kf in enumerate(thinned_keyframes):
                try:
                    lat = float(kf['coordinate']['latitude'])
                    lon = float(kf['coordinate']['longitude'])
                    marker_positions.append((lat, lon))
                    path_positions.append((lat, lon))
                    min_lat, max_lat = min(min_lat, lat), max(max_lat, lat)
                    min_lon, max_lon = min(min_lon, lon), max(max_lon, lon)
                    marker = self.map_widget.set_marker(lat, lon, text=str(i))
                    self.map_markers.append(marker)
                except (KeyError, ValueError, TypeError): continue

            # Draw path
            if len(path_positions) > 1:
                self.map_path = self.map_widget.set_path(path_positions, width=2)

            # Fit map to markers
            if marker_positions:
                 self.map_widget.fit_bounding_box((max_lat, min_lon), (min_lat, max_lon))
            else:
                 self.map_widget.set_position(52.37, 4.89); self.map_widget.set_zoom(10)

            # Add marker for existing POI of this file
            current_poi = self.file_pois.get(json_path)
            if current_poi:
                 if self.poi_marker:
                     self.poi_marker.set_position(current_poi[0], current_poi[1])
                 else:
                     self.poi_marker = self.map_widget.set_marker(current_poi[0], current_poi[1], text="POI", marker_color_circle="red", marker_color_outside="red")
            elif self.poi_marker: # If no POI for this file, remove marker from previous preview
                 self.poi_marker.delete()
                 self.poi_marker = None


            self.update_status(f"Preview ready for {os.path.basename(json_path)}. {info_text}")

        except Exception as e:
            self.update_status(f"Preview Error: {e}")
            messagebox.showerror("Preview Error", f"Could not generate preview:\n{e}")
            if self.map_preview_window and self.map_preview_window.winfo_exists():
                self.map_preview_window.destroy()
            self.map_preview_window = None
            self.map_widget = None


    def convert(self):
        """Handles the conversion process after validation."""
        # Use the list of selected files
        json_files = self.json_files_list
        kmz_path = self.kmz_file_var.get()
        ref_kml_path = self.ref_kml_file_var.get() or None

        # --- Validate Inputs ---
        if not json_files:
            messagebox.showerror("Error", "Please select one or more input JSON files.")
            return
        if not kmz_path:
            messagebox.showerror("Error", "Please specify an output KMZ file path.")
            return
        if ref_kml_path and not os.path.exists(ref_kml_path):
             messagebox.showwarning("Warning", f"Reference KML file not found:\n{ref_kml_path}\n\nA default template will be generated.")
             ref_kml_path = None

        # Validate numeric settings
        desired_waypoints_val = None
        fixed_pitch_val = None
        # center_lat_val = None # Removed global center
        # center_lon_val = None
        manual_heading_val = None
        heading_mode_val = self.heading_mode_var.get()

        try:
            speed = self.speed_var.get()
            turn_damping = self.turn_damping_var.get()
            if speed <= 0 or turn_damping < 0:
                raise ValueError("Speed must be positive and Turn Damping non-negative.")

            desired_waypoints_str = self.desired_waypoints_var.get()
            if desired_waypoints_str:
                desired_waypoints_val = int(desired_waypoints_str)
                if desired_waypoints_val <= 0:
                    raise ValueError("Desired Waypoints must be a positive number.")

            fixed_pitch_str = self.fixed_pitch_var.get()
            if fixed_pitch_str:
                fixed_pitch_val = float(fixed_pitch_str)

            # Validate Per-File POIs if needed
            if heading_mode_val == "Point Towards File's POI":
                missing_poi_files = [os.path.basename(f) for f in json_files if f not in self.file_pois]
                if missing_poi_files:
                    raise ValueError(f"POI coordinates must be set for all files when using 'Point Towards File's POI' mode.\nMissing POI for:\n - " + "\n - ".join(missing_poi_files))

            # Validate Manual Heading if needed
            if heading_mode_val == "Manual Fixed Heading":
                manual_heading_str = self.manual_heading_var.get()
                if not manual_heading_str:
                     raise ValueError("Manual Heading Angle is required for 'Manual Fixed Heading' mode.")
                manual_heading_val = float(manual_heading_str)
                if not (-180 <= manual_heading_val <= 180):
                     raise ValueError("Manual Heading Angle must be between -180 and 180.")

        except ValueError as e:
             messagebox.showerror("Error", f"Invalid setting or missing input: {e}")
             return
        except tk.TclError:
             messagebox.showerror("Error", "Invalid numeric value entered.")
             return


        # --- Prepare Settings ---
        mission_settings = {
            'speed': speed,
            'altitudeType': self.altitude_type_var.get(),
            'finishAction': self.finish_action_var.get(),
            'rcLostAction': self.rc_lost_var.get(),
            'turnDamping': turn_damping,
            'desired_waypoints': desired_waypoints_val,
            'fixed_pitch': fixed_pitch_val,
            'heading_mode': heading_mode_val,
            # 'center_lat': center_lat_val, # Removed
            # 'center_lon': center_lon_val, # Removed
            'manual_heading': manual_heading_val,
            'transitionalSpeed': DEFAULT_TRANSITIONAL_SPEED_MPS,
            'waypointSpeed': DEFAULT_WAYPOINT_SPEED_MPS,
        }

        self.convert_button.config(state=tk.DISABLED)
        self.preview_button.config(state=tk.DISABLED)
        self.update_status("Starting conversion...")

        # --- Run Conversion (pass the list of files and the POI dictionary) ---
        success = ges_json_to_dji_kmz(
            json_files, # Pass the list of selected JSON file paths
            kmz_path,
            ref_kml_path,
            mission_settings,
            self.file_pois, # Pass the dictionary of per-file POIs
            status_callback=self.update_status
        )

        self.convert_button.config(state=tk.NORMAL)
        if tkintermapview is not None and self.file_listbox.curselection(): # Re-enable preview only if library exists AND a file is selected
            self.preview_button.config(state=tk.NORMAL)


        if success:
            messagebox.showinfo("Success", f"KMZ file created successfully!\n\nOutput: {kmz_path}")
            self.update_status("Conversion successful.")
        else:
            messagebox.showerror("Error", "Conversion failed. Check status bar/console for details.")
            # Status bar already shows error


# --- Main execution block for GUI ---
if __name__ == "__main__":
    # Check for library before starting GUI
    if tkintermapview is None:
        root = tk.Tk()
        root.withdraw() # Hide the main empty window
        messagebox.showerror("Dependency Error", "Required library 'tkintermapview' not found.\nPlease install it using:\npip install tkintermapview pillow")
        sys.exit(1) # Exit if library is missing

    root = tk.Tk()
    app = ConverterApp(root)
    root.mainloop()
