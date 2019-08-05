import sys
import re 
import Get_Move as Gm

# command example: 'aodv --animation=true'
if __name__ == '__main__':
    configfile_path = r'grid.config.tcl'
    commandstr = input('Please enter a command：')
    cmdlist = re.split('[\s=-]', commandstr)
    try:
        animation = cmdlist[cmdlist.index('animation') + 1]
    except:
        print("Invalid Input")
        sys.exit(0)        
    if animation != 'true' and animation != 'false':
        print('Animation option error\n')
        sys.exit(0)
    flag = (animation == 'true')
    node_num, sim_time = Gm.get_sim_parameter(configfile_path)
    Gm.control_movement(flag)
