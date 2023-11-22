from pydrake.all import *
import numpy as np

class TemplateController(LeafSystem):
    """
    A simple example of a Drake control system for the Harpy robot. 

    It takes state estimates `x_hat` as input, and outputs target joint angles
    `q_nom`, target joint velocities `v_nom`, feed-forward joint torques `tau_ff`, 
    and thruster forces `thrust`.

                       ----------------
                       |              |
                       |              | ---> x_nom = [q_nom, v_nom]
                       |              |
            x_hat ---> |  Controller  | ---> tau_ff
                       |              |
                       |              | ---> thrust
                       |              |
                       ----------------

    A joint-level PD controller on the robot will compute torques on each joint as

        tau = tau_ff + Kp * (q - q_nom) + Kd * (v - v_nom)
    """
    def __init__(self):
        LeafSystem.__init__(self)

        # Here is where you might want to create an internal MultibodyPlant
        # model of the robot for the controller to use, e.g.,
        # self.plant = ...
        # self.plant_context = ...

        self.input_port = self.DeclareVectorInputPort(
                "x_hat",
                BasicVector(13+13))  # 19 positions, 18 velocities
        self.pos_port = self.DeclareVectorInputPort(
                "position",
                BasicVector(3))
        self.pos_port = self.DeclareVectorInputPort(
                "quaternion",
                BasicVector(4))

        # We'll do some fancy caching stuff so that both outputs can be
        # computed with the same method.
        self._cache = self.DeclareCacheEntry(
                description="controller output cache",
                value_producer=ValueProducer(
                    allocate=lambda: AbstractValue.Make(dict()),
                    calc=lambda context, output: output.set_value(
                        self.CalcOutput(context))))
        
        self.DeclareVectorOutputPort(
                "tau_ff",
                BasicVector(6),  # 3 DoF per leg, plus thruster angle
                lambda context, output: output.set_value(
                    self._cache.Eval(context)["tau_ff"]),
                prerequisites_of_calc={self._cache.ticket()})

        self.DeclareVectorOutputPort(
                "x_nom",
                BasicVector(12),  # actuated positions + velocities
                lambda context, output: output.set_value(
                    self._cache.Eval(context)["x_nom"]),
                prerequisites_of_calc={self._cache.ticket()})

        self.DeclareVectorOutputPort(
                "thrust",
                BasicVector(2),
                lambda context, output: output.set_value(
                    self._cache.Eval(context)["thrust"]),
                prerequisites_of_calc={self._cache.ticket()})

    def CalcOutput(self, context):
        """
        This is where the magic happens. Compute tau_ff, q_nom, and q_nom based
        on the latest state estimate x_hat.

        All the input port data is contained in 'context'. This method must
        return a dictionary with the keys "tau_ff" and "x_nom".
        """
        x_hat = self.EvalVectorInput(context, 0).get_value()
        base_pos = self.EvalVectorInput(context, 1).get_value()
        base_quat = self.EvalVectorInput(context, 2).get_value()

        print(50*"-")
        print("Current Time:")
        print(context.get_time())
        print("Base Position: ")
        print(base_pos)
        print("Base Orientation: ")
        print(base_quat)

        # Target joint angles and velocities
        q_nom = np.array([
            0, 0,   
            0, 0,   
            0, 0])  
        v_nom = np.zeros(6)
        x_nom = np.block([q_nom, v_nom])

        return {"tau_ff": np.zeros(6), "x_nom": x_nom, "thrust": np.array([0, 0])}

