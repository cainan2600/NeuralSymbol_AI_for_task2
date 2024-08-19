import os
import numpy as np
import copy

# 初始化系统状态
self.stage={
    'above_screw':True,
    'near_screw':False,
    'target_aim':True,
    'target_clear':True,
    'clamped':False,
    'part_clamped':False,
    'disassembled':False
}

#define prim
class PrimAction:
    def __init__(self, prim):
        self.pre={}
        self.eff={}
        self.prim=prim
        if self.prim=='insert':
            self.pre={'target_aim':True,'target_clear':True,'above_screw':True}
            self.eff={'above_screw':False,'near_screw':True,'clamped':True}
        elif self.prim=='fumble':
            self.pre={'near_screw':True,'clamped':False,'part_clamped':True}
            self.eff={'clamped':True,'part_clamped':False}
        elif self.prim=='search_and_reinsert':
            self.pre={'near_screw':True,'clamped':False,'part_clamped':False}
            self.eff={'clamped':True}            
        elif self.prim=='disassemble':
            self.pre={'clamped':True}
            self.eff={'disassembled':True}       

    #change stage
    def action(self,stage):

        '''
        用于更新状态（将执行结果状态赋予当前状态，及当前原语执行结束，改变状态，从而进行下一个执行原语的判断）
        '''

        # 深拷贝不影响原列表
        new_stage=copy.deepcopy(stage)
        for e in self.eff:
            new_stage[e]=self.eff[e]
        return new_stage    

    #verify pre of prim//
    def able(self,stage):

        '''
        输入self.stage，即当前状态，循环判断当前状态是否满足某个原语的执行条件
        '''

        for p in self.pre:
            if not stage[p]==self.pre[p]:
                return False
        return True