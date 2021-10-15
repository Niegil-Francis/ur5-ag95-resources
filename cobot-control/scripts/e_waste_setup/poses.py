# Code defining all the poses required for the video
from geometry_msgs.msg import Pose
class Poses:
	def __init__(self):
		
		
		self.poses={}

		self.poses['home']=Pose()
		self.poses['home'].position.x= 1.9265072543217364e-05
		self.poses['home'].position.y= 0.3964500000961947
		self.poses['home'].position.z= 1.0010589997950254
		self.poses['home'].orientation.x= -0.7071067809424444
		self.poses['home'].orientation.y= 2.1272822428359782e-05
		self.poses['home'].orientation.z= -4.184069752087533e-06
		self.poses['home'].orientation.w= 0.7071067810982826


		#---------------------Ethernet poses ---------------------------
		self.poses['eth1_pick']=Pose()
		self.poses['eth1_up']=Pose()
		self.poses['eth2_pick']=Pose()
		self.poses['eth2_up']=Pose()
		self.poses['eth3_pick']=Pose()
		self.poses['eth3_up']=Pose()
		self.poses['eth4_pick']=Pose()
		self.poses['eth4_up']=Pose()
		self.poses['eth5_pick']=Pose()
		self.poses['eth5_up']=Pose()

		#---------------------Bin poses---------------------------
		self.poses['bin_green']=Pose()
		self.poses['bin_green_low']=Pose()
		self.poses['bin_white']=Pose()
		self.poses['bin_cream']=Pose()

		#---------------------Battery poses ---------------------------
		self.poses['batt1_up']=Pose()
		self.poses['batt1_pick']=Pose()
		self.poses['batt2_up']=Pose()
		self.poses['batt2_pick']=Pose()
		self.poses['batt3_up']=Pose()
		self.poses['batt3_pick']=Pose()

		#---------------Deviation poses----------------------------------
		self.poses['dev_1']=Pose()
		self.poses['dev_2']=Pose()

		#---------------------Ethernet poses cart ---------------------------
		self.poses['eth1_pick'].position.x= -0.009940447314517861
		self.poses['eth1_pick'].position.y= 0.42485851442241696
		self.poses['eth1_pick'].position.z= 0.05559952527188834
		self.poses['eth1_pick'].orientation.x= 0.911632111655616
		self.poses['eth1_pick'].orientation.y= -0.40975626239970214
		self.poses['eth1_pick'].orientation.z= -0.026206994765805798
		self.poses['eth1_pick'].orientation.w= 0.01843615599559777


		self.poses['eth1_up'].position.x=    -0.010502079804333209
		self.poses['eth1_up'].position.y=    0.4235458956134686
		self.poses['eth1_up'].position.z=    0.23670076937108014
		self.poses['eth1_up'].orientation.x= 0.9116952896986502
		self.poses['eth1_up'].orientation.y= -0.4097150296317872
		self.poses['eth1_up'].orientation.z= -0.027221623011538323
		self.poses['eth1_up'].orientation.w= 0.014292532166689563

		self.poses['eth2_pick'].position.x= 0.33063872483940876
		self.poses['eth2_pick'].position.y= 0.07331739400068785
		self.poses['eth2_pick'].position.z= 0.0590757670898397
		self.poses['eth2_pick'].orientation.x= -0.36671038318775634
		self.poses['eth2_pick'].orientation.y= 0.9297519377407754
		self.poses['eth2_pick'].orientation.z= 0.031696348618164116
		self.poses['eth2_pick'].orientation.w= 0.00895380443377027

		self.poses['eth2_up'].position.x=    0.3046667209646727
		self.poses['eth2_up'].position.y=    0.07097298628749228
		self.poses['eth2_up'].position.z=    0.4
		self.poses['eth2_up'].orientation.x= 0.3666959293994116
		self.poses['eth2_up'].orientation.y= -0.9299609518192695
		self.poses['eth2_up'].orientation.z= -0.026225482623359893
		self.poses['eth2_up'].orientation.w= 0.004352874277051529

		self.poses['eth3_pick'].position.x= 0.17456217932554524
		self.poses['eth3_pick'].position.y= 0.23494716128988424
		self.poses['eth3_pick'].position.z= 0.06296836917202389
		self.poses['eth3_pick'].orientation.x= -0.9164356772954013
		self.poses['eth3_pick'].orientation.y= 0.39836904174110344
		self.poses['eth3_pick'].orientation.z= -0.0019522580087129944
		self.poses['eth3_pick'].orientation.w= 0.037999271717515326

		self.poses['eth3_up'].position.x=    0.14589578957155136
		self.poses['eth3_up'].position.y=    0.21561099067589198
		self.poses['eth3_up'].position.z=    0.29725875502433285
		self.poses['eth3_up'].orientation.x= -0.9167227316880618
		self.poses['eth3_up'].orientation.y= 0.39916928158940196
		self.poses['eth3_up'].orientation.z= 0.010216881903743489
		self.poses['eth3_up'].orientation.w= 0.013376590221111111


		self.poses['eth4_pick'].position.x= 0.6286517015217976
		self.poses['eth4_pick'].position.y= 0.1538748831628606
		self.poses['eth4_pick'].position.z= 0.06785450270437571
		self.poses['eth4_pick'].orientation.x= -0.9160279300854846
		self.poses['eth4_pick'].orientation.y= 0.40003325314951677
		self.poses['eth4_pick'].orientation.z= -0.02915843514188484
		self.poses['eth4_pick'].orientation.w= 0.004001666901894628


		self.poses['eth4_up'].position.x=    0.6327953971520721
		self.poses['eth4_up'].position.y=    0.15316116513086828
		self.poses['eth4_up'].position.z=    0.16035234064986983
		self.poses['eth4_up'].orientation.x= -0.9247651832833177
		self.poses['eth4_up'].orientation.y= 0.3802081179973708
		self.poses['eth4_up'].orientation.z= 0.01028280815849552
		self.poses['eth4_up'].orientation.w= 0.01205846807206317
		
		self.poses['eth5_up'].position.x=    0.384027006160421
		self.poses['eth5_up'].position.y=   0.2967614717173512
		self.poses['eth5_up'].position.z=    0.15
		self.poses['eth5_up'].orientation.x= -0.9341783365270028
		self.poses['eth5_up'].orientation.y= 0.35660552730738715
		self.poses['eth5_up'].orientation.z= 0.00550669523836223
		self.poses['eth5_up'].orientation.w= 0.01063060511043202

		self.poses['eth5_pick'].position.x=   0.38224329559763076
		self.poses['eth5_pick'].position.y=   0.2970265193626293
		self.poses['eth5_pick'].position.z=   0.06402909404908887
		self.poses['eth5_pick'].orientation.x= 0.926922265021174
		self.poses['eth5_pick'].orientation.y= -0.374647603261073
		self.poses['eth5_pick'].orientation.z= -0.017915147120697104
		self.poses['eth5_pick'].orientation.w= 0.0115470984404672 

		#---------------------Battery poses cart ---------------------------
		self.poses['batt1_pick'].position.x= 0.29821450236668545
		self.poses['batt1_pick'].position.y= -0.20650243570091836
		self.poses['batt1_pick'].position.z= 0.03914969710137903
		self.poses['batt1_pick'].orientation.x= 0.40983360948083386
		self.poses['batt1_pick'].orientation.y= 0.9116487233498983
		self.poses['batt1_pick'].orientation.z= 0.0020658213157145207
		self.poses['batt1_pick'].orientation.w= 0.030475402158191386


		self.poses['batt1_up'].position.x=    0.2978840403464875
		self.poses['batt1_up'].position.y=    -0.2091579922646466
		self.poses['batt1_up'].position.z=    0.4
		self.poses['batt1_up'].orientation.x= -0.9167227316880618
		self.poses['batt1_up'].orientation.y= 0.39916928158940196
		self.poses['batt1_up'].orientation.z= 0.010216881903743489
		self.poses['batt1_up'].orientation.w= 0.013376590221111111

		self.poses['batt2_pick'].position.x= 0.31346104305565564
		self.poses['batt2_pick'].position.y= -0.18658692188256198
		self.poses['batt2_pick'].position.z= 0.03745711831045184
		self.poses['batt2_pick'].orientation.x= -0.915566499790814
		self.poses['batt2_pick'].orientation.y= 0.401779568348251
		self.poses['batt2_pick'].orientation.z= -0.01523530244170295
		self.poses['batt2_pick'].orientation.w= 0.008890921110909734

		self.poses['batt2_up'].position.x=    0.3113654053831539
		self.poses['batt2_up'].position.y=    -0.1823330349938246
		self.poses['batt2_up'].position.z=    0.06986040488439596
		self.poses['batt2_up'].orientation.x= -0.9155746668334579
		self.poses['batt2_up'].orientation.y= 0.40178940042398403
		self.poses['batt2_up'].orientation.z= -0.010193712157250748
		self.poses['batt2_up'].orientation.w= 0.01357922649464934
		
		
		#---------------------Battery poses cart ---------------------------
		self.poses['bin_green'].position.x= -0.09596114852499464
		self.poses['bin_green'].position.y= -0.5441958279257513
		self.poses['bin_green'].position.z= 0.5
		self.poses['bin_green'].orientation.x= 0.39183171372010445
		self.poses['bin_green'].orientation.y= 0.9186293109561782
		self.poses['bin_green'].orientation.z= -0.030851989877057784
		self.poses['bin_green'].orientation.w= 0.04045061057597806

		self.poses['bin_green_low'].position.x= -0.11
		self.poses['bin_green_low'].position.y= -0.5441958279257513
		self.poses['bin_green_low'].position.z= 0.1
		self.poses['bin_green_low'].orientation.x= 0.39183171372010445
		self.poses['bin_green_low'].orientation.y= 0.9186293109561782
		self.poses['bin_green_low'].orientation.z= -0.030851989877057784
		self.poses['bin_green_low'].orientation.w= 0.04045061057597806
		
		
		self.poses['bin_white'].position.x=    -0.2946566105261906
		self.poses['bin_white'].position.y=    -0.367029099707315
		self.poses['bin_white'].position.z=    0.13469777151292003
		self.poses['bin_white'].orientation.x= 0.3941773756843475
		self.poses['bin_white'].orientation.y= 0.9177074889905723
		self.poses['bin_white'].orientation.z= -0.036784018679140884
		self.poses['bin_white'].orientation.w= 0.03292563012353585
		
		
		self.poses['bin_cream'].position.x= -0.48601383721278946
		self.poses['bin_cream'].position.y= -0.17541243426768377
		self.poses['bin_cream'].position.z= 0.11183938343572691
		self.poses['bin_cream'].orientation.x= 0.40952565924562395
		self.poses['bin_cream'].orientation.y= 0.9103296539495324
		self.poses['bin_cream'].orientation.z= -0.05162756124972479
		self.poses['bin_cream'].orientation.w= 0.03038503709011495		

		#------------------------trajectory deviation-----------------------------
			
		self.poses['dev_1'].position.x=    0.44966037144216975
		self.poses['dev_1'].position.y=    0.08955934808983079
		self.poses['dev_1'].position.z=    0.11019792892981542
		self.poses['dev_1'].orientation.x= -0.8894146396292348
		self.poses['dev_1'].orientation.y= 0.45535139159604926
		self.poses['dev_1'].orientation.z= 0.022426375171301617
		self.poses['dev_1'].orientation.w= 0.03307214358664836

		self.poses['dev_2'].position.x= 0.3740768861778521
		self.poses['dev_2'].position.y= 0.24601602202200754
		self.poses['dev_2'].position.z= 0.18487855505915238
		self.poses['dev_2'].orientation.x= -0.8702574177961903
		self.poses['dev_2'].orientation.y= 0.4895726985728601
		self.poses['dev_2'].orientation.z= 0.03760919511592319
		self.poses['dev_2'].orientation.w= 0.03944804210004537

		#-----------------------------------------------------------------------




	def pose_call(self,name):
		return self.poses[name]