from scipy.spatial.transform import Rotation as R
q1=[0.01674002,-0.00373753,0,0.99985289]
q2=[ 0.0090983,0,0.8596388,0.51082125]
z_axis_q1 = R.from_quat(q2).apply([0, 0, 1])
z = R.from_quat(q1).apply([0, 0, 1])
print(z_axis_q1)