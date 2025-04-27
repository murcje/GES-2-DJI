# GES-2-DJI
Convert Google Earth Studio camera animations to DJI waypointsfile

Fly a camera around a location in GES and duplicate that flight in the real world using a DJI drone. Get info on total flight distance and estimated flight time.

*This is all expiremental work and I take no responsibility if any damages or harm occur by using this script.*

For planning of flight plans for drone survey or capturing imagery for photogrammetry or 3DGS it can be useful to pre-visualize the flight path. 
With this script you can convert GES .json fils to a DJI waypoints file.

At the moment, it is mainly intended for orbital flight plans. Usually you would fly multiple orbits at different altitudes. Conversion from GES eular rotations to DJI heading and pitch is not yet working so these need to be set manually before conversion. Instructions below.

SINGLE ORBIT WORKFLOW:

1   Plan your (orbital) flight in GES. First set your field of view to correspond with that of your drone (90 for the DJI Mini 4 Pro). Set keyframes or start from a new Orbit project.

2   Export 3D tracking data, choose .json as a file type.

3   Execute the script to load the gui by opening a CMD prompt and running:

    python.exe GES2DJI.py
    
4   Add the .json file and adjust the fields in Mission Settings to your needs. 
- Altitude type is best set to WGS86.
- Desired Waypoints is used to thin the keyframes from GES. If not used, you will get many waypoints on your drone.
- Fixed Gimbal Pitch is for adjusting the gimbal vertical tilt.

In GES, 90 degrees is looking at the horizon and 0 is looking down. For DJI, the horizon is at 0 degrees and looking down is -90 degrees. Adjust your desired pitch angle accordingly.

5   Select the Heading Mode - For orbital flight around a poi, choose Point Towards Files's POI.

6   Select your file in the list and press Preview Selected File & Set POI

7   Set the desired POI location on the map (usually in the center of the orbit) and close the map window.

8   Press Set POI for Selected

8   Press Convert & Merge to KMZ to start the process.


MULTIPLE ORBIT WORKFLOW:

1   Plan each of your flights seperately in GES. First set your field of view to correspond with that of your drone (90 for the DJI Mini 4 Pro). Set keyframes or start from a new Orbit project.

2   Export 3D tracking data, choose .json as a file type. Do this seperately for each flight.

3   Execute the script to load the gui by opening a CMD prompt and running:

    python.exe GES2DJI.py
    
4   Add the .json files and adjust the fields in Mission Settings to your needs. 
- Altitude type is best set to WGS86.
- Desired Waypoints is used to thin the keyframes from GES. If not used, you will get many waypoints on your drone.
- Fixed Gimbal Pitch is for adjusting the gimbal vertical tilt.

In GES, 90 degrees is looking at the horizon and 0 is looking down. For DJI, the horizon is at 0 degrees and looking down is -90 degrees. Adjust your desired pitch angle accordingly.

5   Select the Heading Mode - For orbital flight around a poi, choose Point Towards Files's POI.

6   Select the first file in the list and press Preview Selected File & Set POI

7   Set the desired POI location on the map (usually in the center of the orbit) and close the map window.

8   Press Set POI for Selected

9   Select the second file in the list and press Preview Selected File & Set POI

10  Set the desired POI location on the map (usually in the center of the orbit) and close the map window.

11  Press Set POI for Selected - continue this for all .json files

12  Press Convert & Merge to KMZ to start the process.


Copying your waypointfile to the remote:

Connect the remote to a computer and browse to: 

android/data/dji.go.v5/files/waypoint. 

Here you will see your saved waypoint files. You will need to overwrite an existing one. Rename your converted .kmz file as an existing waypoint file on your remote and overwrite it (perhaps make a copy of the original if you want to save it).

Load the waypoint file on you remote. The name will be the saame as the original waypointfile that you have overwritten. The amount of waypoints and length will be updated in the description as soon as you open and save the file on your remote. Check the flight path in the map mode of the remote to make sure all looks good before flying.

*This is all expiremental work and I take no responsibility if any damages or harm occur by using this script.*

