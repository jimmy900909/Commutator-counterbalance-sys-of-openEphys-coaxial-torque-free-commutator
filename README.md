This is a wireless counterbalance system that can adjust the length of the cable automatically for OpenEphys commutator while conducting freely moving animal experiment

#closed-loop Automatic Counterbalance animal experiment system

#tension-free and biting-prevention for cable in the freely moving animal experiment

The toggling and dragging can seriously cause the artifacts to signal when doing a cable-connected behavior experiment on an awake and freely-moving animal. The automatic rotation commutator is widely used in labs and commercialized, however, to adjust the length of the cable is always depends on manual holding or passive counterbalance sys. As the hanged part would be bited by animal and the tension can constraint the movement, a active counterbalance system is needed for such animal experiment.

This system consist of two part, openEphys(OE) commutator for torque-free compensation, and conterbalance system for tension control. 

Rotation commutator recieve the orientation data of the markers(rigid body) from Motive and send it to the commutator through Natnet SDK(https://optitrack.com/software/natnet-sdk)
The python code ui.py send the command that rotate the same degree as the target does to the commutator.

The counterbalance system detect the tension of the thread connected to the data acquisition cable, retract or release the thread to control the tension in a threshold.  

Following is the Assembly instructions of the mechanical structure. The CAD files are under counterbalance/mechanical design 
<img width="1003" height="736" alt="image" src="https://github.com/user-attachments/assets/a5795300-2385-4609-858a-f02fced355d5" />

User guide
1.connect the OE commutator to the PC through the serial port(USB)
2.Open motive, select the reigid body and enable streaming
3.Run ui.py in the under the Natnet python client
4.Pause the rotation compensation first 
5.Set up the the counterbalance system by plug in the power, at the set time leave the thread from the loadcell to start from 0 weight.
6.start the rotation commutator 
