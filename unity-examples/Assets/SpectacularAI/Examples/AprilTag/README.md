# April Tag with SLAM relocalization

This example demonstrates using an April Tag to align a saved SLAM map with a digital twin of the mapped environment in Unity.

Note that relocalization is still an experimental feature of the SDK. The system has trouble relocalizing from arbitrary positions in the map. Therefore, we recommend relocalizing by repeating the first part of your recording path (see step 10).

## Instructions

1. Print and place an April Tag somewhere in your environment. You can use an [online generator](https://chaitanyantr.github.io/apriltag.html) to create the tag.

2. Record a walk around the room using the [Recording](https://spectacularai.github.io/docs/sdk/recording.html). You need to enable the SLAM map export, so run it as 
    ```bash
    sai-cli record oak --map
    ```
    This will create a directory `data/TIMESTAMP`, where TIMESTAMP is the date and time that the recording started. 

3. Process the recording into a mesh using the command line tool. Replace PATH_TO_DATA with the directory created in the previous step.
    ```bash
    sai-cli process PATH_TO_DATA mesh.obj --preview3d --texturize
    ```
    This will create the `mesh.obj` file and its texture files, which can be imported directly to Unity.  

4. In Unity, clone the AprilTag/Scenes/AprilTag scene, remove the Scene/RoomWithAprilTags node, and import the mesh from the previous step.  

5. Move and rotate the AprilTag prefab in Scene/Tags so that it matches as closely as possible its real-world position. Make sure to set the tag id. family and size correctly in the component inspector.  

6. In the AprilTagController inspector, click the "Serialize April Tags" button. This will generate a `tags.json` file and open the file browser to the directory where it is saved. Move the `tags.json` file to the recording output directory from step 2.  

7. Clone or download the [SpectacularAI/sdk-examples](https://github.com/SpectacularAI/sdk-examples) repository and in the `python/oak/` directory, run:
    ```bash
    python relocalize_apriltags.py PATH_TO_RECORDING_DIR
    ```
    This will find the alignment between the SLAM world and the Unity world using the April Tag pose data as an anchor point. It saves the alignment as a transform matrix, in a file named `slam_config.json` that it saves in the recording directory you passed as an argument to the script.  

8. Place the PATH_TO_RECORDING_DIR (or whatever you used as an argument in step 7) into the "Slam Data Path" field of the inspector of the VIO prefab in the Unity scene.  

9. Remove the April Tag from your environment.

10. Press Play in Unity and slowly repeat the same movements that you made at the beginning of your recording path. If everything workd, the system should relocalize within a few seconds.
