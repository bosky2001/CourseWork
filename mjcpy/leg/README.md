# Legged Hopping Robot
An FSM calls in the PD controller to maintain a particular pose in flight and an active damping controller for stance. After each cycle, a new touchdown angle is calculated using raibert hopping, and the pose is changed. Also implemented is the FSM with a momentum observer, proprioceptive methods, and change in length used to detect contact.
