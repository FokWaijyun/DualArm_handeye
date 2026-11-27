(handeye) huo@huo-thinkpad:~/project/DualArm_handeye$ python scripts/x5_handeye_calibration.py
Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[INFO] [1795674176.025216451] [x5_handeye_calibration_viewer]: 已订阅标定压缩彩色图像：/camera_01
[2026-11-26 14:23:15.248] [xapi-info ] [logger.cpp:63 Logger] Logger
[2026-11-26 14:23:15.248] [xapi-info ] [logger.cpp:149 init] init
[2026-11-26 14:23:15.248] [xapi-info ] [logger.cpp:545 parseIniFileAndSetupLogger] Logger(by xapi.ini) is expired!!! 
[xapi-debug] XEnableDebugOutput<Disable>
[xapi-debug] XEnableDebugOutput<Disable>
[xzmq-info ] prefetchAndProcessPendingMessages: Pre-fetching lua buffer data...
[xzmq-info ] prefetchAndProcessPendingMessages: Pre-fetching free buffer data...
[xzmq-info ] prefetchAndProcessPendingMessages: Pre-fetching visual buffer data...
Traceback (most recent call last):
  File "/home/huo/project/DualArm_handeye/scripts/x5_handeye_calibration.py", line 657, in _refresh
    detection = detect_chessboard(
  File "/home/huo/project/DualArm_handeye/scripts/x5_handeye_calibration.py", line 1246, in detect_chessboard
    error = reprojection_error(object_points, corners, rvec, tvec, camera_matrix, dist)
  File "/home/huo/project/DualArm_handeye/scripts/x5_handeye_calibration.py", line 1677, in reprojection_error
    return float(cv2.norm(image_points, projected, cv2.NORM_L2) / len(projected))
cv2.error: OpenCV(5.0.0) /home/conda/feedstock_root/build_artifacts/libopencv_1786395764748/work/modules/core/src/norm.dispatch.cpp:525: error: (-2:Unspecified error) in function 'double cv::norm(InputArray, InputArray, int, InputArray)'
> Input type mismatch (expected: '_src1.type() == _src2.type()'), where
>     '_src1.type()' is 5 (CV_32FC1)
> must be equal to
>     '_src2.type()' is 37 (CV_32FC2)
已中止 (核心已转储)


从报错信息看到，是相机看到完整棋盘格子后，才出现的报错；棋盘格出现前，UI界面的使用和脚本都没有任何问题
我怀疑是视觉相关的代码引起的报错退出

