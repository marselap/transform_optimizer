
install apriltag detection:
https://github.com/pupil-labs/apriltags.git 


cd /path/to/scripts

chmod +x optimize_class.py

1. Test can be run as 

./optimize_class.py

uses poses_ex.txt (list of detected target positions in camera frame)
and tfs_ex.txt (list of e.g. marker poses in global frame)

does optimization from zero initial guess and one manual

2. to do dataset collection, setup stuff in sync.py (global frame, marker frame in /tf, camera topics)
run the node

when camera set into a new pose (where target is visible) pub a message to topic /record_apriltag
when enough poses are recorded (e.g.5-10), pub a msg to /apriltags_done. This saves the poses and tfs files

now you can run optimize_class