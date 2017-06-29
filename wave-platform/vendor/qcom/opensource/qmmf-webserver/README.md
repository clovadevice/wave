#Web service implementation which exposes the QMMF framework API

#Testing
 You can use the following 'curl' commands from any host connected to the target device. Currently support is present for:
- Query status (This can be called in any state)
>curl 'http://device-ip:4000' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' -i

- Connect to QMMF recorder
>curl 'http://device-ip:4000/connect' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i

- FR rule add
>curl 'http://<device-ip>:4000/vamconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b58","event_type":8,"name":"face recognized","sensitivity":1,"status":1}], "zones" :  [{"id" : "d4668d05-faa3-zone-0001-fde575fa6b58","type" : 2,"points" : [{"x" : 100,"y" : 100}, {"x" : 100,"y" : 999}]}]}' -i

- FR rule remove
>curl 'http://<device-ip>:4000/vamremoveconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b58","event_type":8,"name":"face recognized","sensitivity":1,"status":1}]}' -i

- FD rules add
>curl 'http://<device-ip>:4000/vamconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b59","event_type":7,"name":"face recognized","sensitivity":1,"status":1}], "zones" :  [{"id" : "d4668d05-faa3-zone-0001-fde575fa6b59","type" : 2,"points" : [{"x" : 100,"y" : 100}, {"x" : 100,"y" : 999}]}]}' -i

- FD rule remove
>curl 'http://<device-ip>:4000/vamremoveconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b59","event_type":7,"name":"face recognized","sensitivity":1,"status":1}]}' -i


- Enroll image to VAM
>(echo -n "vam_enroll_object_type=3&vam_enroll_event_type=8&vam_enroll_format=7&vam_enroll_width=320&vam_enroll_height=400&vam_enroll_id=1&vam_enroll_name=testface&vam_enroll_image_id=1&vam_enroll_data="; cat <path-to-base64-url-encoded_grayscale_face>) | curl --data @- 'http://device-ip:4000/vamenroll' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' -i

- Start camera
>curl 'http://device-ip:4000/startcamera' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'zsl_mode=0&zsl_queue_depth=10&zsl_width=1920&zsl_height=1080&framerate=30&flags=0&camera_id=0' -i

- Capture an image
>curl 'http://device-ip:4000/captureimage' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0&image_width=4000&image_height=3000&image_quality=95' -i

- Create a session
>curl 'http://device-ip:4000/createsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i

- Create a video track
>curl 'http://device-ip:4000/createvideotrack' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0&track_id=1&session_id=1&track_width=320&track_height=240&track_codec=1&bitrate=512000&framerate=30&track_output=0&low_power_mode=0' -i

- Start a session
>curl 'http://device-ip:4000/startsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1' -i

- Configure camera parameters
>curl 'http://<device-ip>:4000/setcameraparam' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0&ir_mode=on' -i

- Create user text overlay
>curl 'http://<device-ip>:4000/createoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_type=usertext&ov_position=topleft&ov_color=869007615&ov_user_text=test' -i

- Create datetime overlay
>curl 'http://<device-ip>:4000/createoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_type=datetime&ov_position=topright&ov_color=869007615&ov_date=mmddyyyy&ov_time=hhmmss_24hr' -i

- Create bounding box overlay
>curl 'http://<device-ip>:4000/createoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_type=boundingbox&ov_box_name=test&ov_start_x=100&ov_start_y=100&ov_width=100&ov_height=100&ov_color=869007615' -i

- Delete overlay
>curl 'http://<device-ip>:4000/deleteoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Remove overlay
>curl 'http://<device-ip>:4000/removeoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Set overlay
>curl 'http://<device-ip>:4000/setoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Get overlay
>curl 'http://<device-ip>:4000/getoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Update overlay
>curl 'http://<device-ip>:4000/updateoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1&ov_type=boundingbox&ov_box_name=slack&ov_start_x=100&ov_start_y=100&ov_width=100&ov_height=100&ov_color=869007615' -i

- Stop a session
>curl 'http://device-ip:4000/stopsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1&flush=1' -i

- Delete a video track
>curl 'http://device-ip:4000/deletevideotrack' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1&track_id=1' -i

- Delete a session
>curl 'http://device-ip:4000/deletesession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1' -i

- Stop camera
>curl 'http://device-ip:4000/stopcamera' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0' -i

- Disconnect from QMMF recorder
>curl 'http://device-ip:4000/disconnect' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i

#API summary
'GET' requests to the server root will return a JSON response containing information about the server, camera capabilities, created streams, active RTSP urls etc. The JSON reply will have the following example structure:
```json
{
    "Tracks": [{
        "bitrate":10000000,
        "camera_id":0,
        "codec":1,
        "framerate":30,
        "height":1080,
        "output":0,
        "rtsp_url":"rtsp://<device-ip>:8900/live",
        "session_id":1,
        "track_id":1,
        "width":1920
    }],
    "AudioTracks":null,
    "Cameras":[{
        "camera_id":0,
        "flags":0,
        "framerate":30,
        "hdr_modes":" off on",
        "ir_modes":" off on",
        "nr_modes":" off fast high-quality minimal zsl",
        "zsl_height":1080,
        "zsl_mode":0,
        "zsl_q_depth":8,
        "zsl_width":1920
    }]
}
```

* __connect__ - Connects to the QMMF recorder service.
 * Arguments - None.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __vamconfig__ - Set VAM configuration. The configuration will get applied only in case VAM is active otherwise it will return error.
 * Arguments
   * vam_config - JSON array containing the VAM configuration. Check 'curl' example for reference.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __vamremoveconfig__ - Remove VAM configuration. The configuration will removed only in case VAM is active otherwise it will return error.
 * Arguments
   * vam_config - JSON array containing the VAM configuration. Check 'curl' example for reference.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __vamenroll__ - VAM image enroll. This should get called only after a session with YUV track and VAM output gets started.
 * Arguments
   * vam_enroll_object_type - Enroll object type.
   * vam_enroll_event_type - Enroll event type.
   * vam_enroll_format - Format of the enrolled image.
   * vam_enroll_width - Width of the enrolled image.
   * vam_enroll_height - Height of the enrolled image.
   * vam_enroll_id - Enroll id.
   * vam_enroll_name - Name of the enrolled frame.
   * vam_enroll_image_id - Id of the enrolled image.
   * vam_enroll_data - Base64 url encoded data of the enrolled image.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __startcamera__ - Starts a speficic camera.
 * Arguments
   * camera_id - Index of the camera that needs to be started.
    * zsl_mode - Enable ZSL. Possible values [0,1]
    * zsl_queue_depth - ZSL queue depth.
    * zsl_width - Width of the buffers used by ZSL.
    * zsl_height - Height of the buffers used by ZSL.
    * framerate - Framerate with which ZSL will run.
    * flags - Additional flags. Not used currently.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __captureimage__ - Takes a camera snapshot. The resulting Jpeg will get stored locally in the device '/data' folder.
 * Arguments
   * camera_id - Index of the camera that needs to capture an image.
   * image_width - Width of the captured image.
   * image_height - Height of the captured image.
   * image_quality - Jpeg compression. Maximum value is 100.
 * Returns JSON structure containing the Base64 encoded snapshot in the "Data" key and timestamp in "Timestamp". If an error occurs a status and error JSON structure will get sent back.
* __createsession__ - Creates a new recording session.
 * Arguments - None
 * In case of success it returns the session id in json structure. The field which contains the value will be called "SessionId" and the value will be non-negative. In case of error it will return a JSON status and error.
* __createaudiotrack__ - Creates a new audio track within a give recording session.
 * Arguments
   * sample_rate - Audio stream sample rate.
    * track_id - A unique positive track_id.
    * session_id - The session id in which the track will run. The session id should match a valid session created by a previous call to "createsession".
    * num_channels - The audio channel count.
    * bit_depth - Bits per sample.
    * track_codec - Codec to be used for this video track. Values should be within this range {0 (->PCM), 1 (->AAC), 2 (->AMR), 3 (->AMRWB)}.
    * bitrate - In case of codecs different from PCM.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deleteaudiotrack__ - Deletes a given audio track within a recording session.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
   * track_id - The id of track that needs to be removed.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __createvideotrack__ - Creates a new video track within a give recording session.
 * Arguments
   * camera_id - Index of the camera that should be used as input for the track.
    * track_id - A unique positive track_id.
    * session_id - The session id in which the track will run. The session id should match a valid session created by a previous call to "createsession".
    * track_width - The width of the video track.
    * track_height - The height of the video track.
    * track_codec - Codec to be used for this video track. Values should be within this range {0 (->HEVC), 1 (->AVC), 2 (->YUV), 3 (->RDI), 4 (->RAWIDEAL)}.
    * bitrate - In case of HEVC or AVC
    * framerate - Framerate with which the track will run.
    * low_power_mode - Enables low power mode (preview camera stream). Possible values {0 (Default disabled), 1 (Enabled)}.
    * track_output - The sink which will receive, process and potentially stream the track data. Values should be within this range {0 (->RTSP streaming), 1 (->Input in VAM), 2 (->MP4 muxing, stored in the local device '/data' directory), 3 (->3GP muxing, stored in the local device '/data' directory), 4 (->MpegTs muxing and streaming over RTSP), 5 (->RTMP streaming)}
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __startsession__ - Starts a particular recording session. All tracks added inside this session will start too.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __setcameraparam__ - Sets specific camera parameters. This specific request can be made after a session with at least one video track get started.
 * Arguments
   * camera_id - Index of the camera on which the parameters should get applied.
   *ir_mode - Infra red mode one of {off, on}. Please query camera status for supported values.
   *hdr_mode - HDR mode one of {off, on}. Please query camera status for supported values.
   *nr_mode - Noise reduction mode one of {off, fast, high-quality, minimal, zsl}. Please query camera status for supported values.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __createoverlay__ - Create video overlay. It can be called after initialization.
 * Arguments
   * ov_type - Type of overlay. It should be one of {datetime,usertext,staticimage,boundingbox,privacymask}
    * track_id - The track id to which this overlay will get attached.
    * ov_color - Text color value. Required for overlays of type {datetime,usertext,boundingbox,privacymask}
    * ov_position - Overlay position. It should be one of {topleft,topright,center,bottomleft,bottomright,none}. Required for overlays of type {datetime,usertext,staticimage}
    * ov_user_text - User defined text. Required for overlays of type {usertext}
    * ov_start_x - Overlay box starting X position. Required for overlays of type {boundingbox, privacymask}
    * ov_start_y - Overlay box starting Y position. Required for overlays of type {boundingbox, privacymask}
    * ov_width - Overlay width. Required for overlays of type {staticimage, boundingbox, privacymask}
    * ov_height - Overlay height. Required for overlays of type {staticimage, boundingbox, privacymask}
    * ov_image_location - Location of YUV image on device file system. Required for overlays of type {staticimage}
    * ov_box_name - Overlay bounding box title name. Required for overlays of type {boundingbox}
    * ov_date - Date format. It should be one of {yyyymmdd,mmddyyyy}. Required for overlays of type {datetime}
    * ov_time - Time format. It should be one of {hhmmss_24hr,hhmmss_ampm,hhmm_24hr,hhmm_ampm}. Required for overlays of type {datetime}
 * Returns JSON status and error. If error equals 'none', status will contain the Overlay ID, which can be used to reference the Overlay in subsequent overlay related operations.
* __updateoverlay__ - Update the parameters of already created overlay.
 * Arguments
   * ov_type - Type of overlay. It should be one of {datetime,usertext,staticimage,boundingbox,privacymask}
    * ov_id - Overlay id received during create.
    * track_id - The track id to which this overlay will get attached.
    * ov_color - Text color value. Required for overlays of type {datetime,usertext,boundingbox,privacymask}
    * ov_position - Overlay position. It should be one of {topleft,topright,center,bottomleft,bottomright,none}. Required for overlays of type {datetime,usertext,staticimage}
    * ov_user_text - User defined text. Required for overlays of type {usertext}
    * ov_start_x - Overlay box starting X position. Required for overlays of type {boundingbox, privacymask}
    * ov_start_y - Overlay box starting Y position. Required for overlays of type {boundingbox, privacymask}
    * ov_width - Overlay width. Required for overlays of type {staticimage, boundingbox, privacymask}
    * ov_height - Overlay height. Required for overlays of type {staticimage, boundingbox, privacymask}
    * ov_image_location - Location of YUV image on device file system. Required for overlays of type {staticimage}
    * ov_box_name - Overlay bounding box title name. Required for overlays of type {boundingbox}
    * ov_date - Date format. It should be one of {yyyymmdd,mmddyyyy}. Required for overlays of type {datetime}
    * ov_time - Time format. It should be one of {hhmmss_24hr,hhmmss_ampm,hhmm_24hr,hhmm_ampm}. Required for overlays of type {datetime}
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __getoverlay__ - Retrieve parameters of specific overlay.
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id to which this overlay will get attached.
 * On success returns a JSON object with all overlay parameters. For example a static image overlay could return:
```json
{
    "ov_color":869007615,
    "ov_date":"mmddyyyy",
    "ov_position":"topleft",
    "ov_time":"hhmmss_24hr",
    "ov_type":"datetime"
}
```
* __setoverlay__ - Set a particular overlay per track
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id to which this overlay will get attached.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __removeoverlay__ - Remove a particular overlay from give track
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id from which this overlay will get removed.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deleteoverlay__ - Deletes a specific overlay.
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id from which this overlay will get delete.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __stopsession__ - Stops a particular recording session and all tracks contained within.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
    * flush - Valid values are in the range of {0 (->Don't discard any pending buffers in the pipeline), 1 (->Discard all buffers in the pipeline)}.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deletevideotrack__ - Deletes a given video track within a recording session.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
   * track_id - The id of track that needs to be removed.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deletesession__ - Removes a particular recording session.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __stopcamera__ - Stops a given camera or cameras.
 * Arguments
   * camera_id - Index of the camera that needs to get stopped.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __disconnect__ - Disconnects from the QMMF recorder.
 * Arguments - none
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.

