from queue import Queue


ori_stage={'above_screw':True,
    'near_screw':False,
    'target_aim':False, 
    'target_clear':True,
    'clamped':False,
    'part_clamped':False,
    'disassembled':False}   

pathQueue=Queue(0)

for i in range(3):
    print(i)
    pathQueue.put([i])
    # print(pathQueue.get())
    # pathQueue.get()
print(pathQueue)