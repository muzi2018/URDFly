from .sympybotics import RobotDef, RobotDynCode, robotcodegen

def create_gx7(mdh_list):
    mdh_list = [[m[3], m[2], m[1], m[0]] for m in mdh_list]
    rbtdef = RobotDef(
                "rbt_name",
                mdh_list,
                dh_convention="mdh"
            )

    rbtdef.frictionmodel = {'Coulomb', 'viscous', 'offset'}
    rbtdef.driveinertiamodel = 'simplified'

    rbt = RobotDynCode(rbtdef)
    rbt.calc_base_parms()

    regressor_func_str = robotcodegen.robot_code_to_func('C', rbt.H_code, 'regressor', 'H_func', rbtdef)
    return regressor_func_str, rbt.dyn.base_idxs
    












