#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from dobot_msgs_v4.srv import ServoJ
from dobot_msgs_v4.srv import GetAngle, SetFloat
import math


class ServoJClient(Node):
    def __init__(self):
        super().__init__('node_servoj')

        #1.) tu
        self.cli_move = self.create_client(ServoJ, '/dobot_bringup_ros2/srv/ServoJ')#servJ client
        self.cli_get = self.create_client(GetAngle, '/dobot_bringup_ros2/srv/GetAngle')#getAngle client
        self.cli_json = self.create_client(SetFloat, '/nalozi_tocke')#nalozi_tocke client
        
        
        while not self.cli_get.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Čakam na GetAngle service...')
        while not self.cli_move.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Čakam na ServoJ service...')
        while not self.cli_json.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Čakam na nalozi_tocke service...')

        self.waypoints = []
        self.have_waypoints = False


        # self.waypoints = [
        #     [138.26, 37.39, -49.56, 72.85, 20.55, -27.74],  
        #     [145.00, 45.00, -42.00, 70.00, 25.00, -20.00],   
        #     [150.00, 38.00, -50.00, 80.00, 30.00, -15.00],  
        #     [135.00, 47.00, -40.00, 65.00, 18.00, -35.00],
        # ]

        self.index = 0
        #self.max_hitrost = np.array([20, 20, 20, 20, 20, 20], dtype=float) # maksimalna hitrost zasuka v stopinjah na sekundo
        self.max_hitrost = 20.0 # maksimalna hitrost zasuka v stopinjah na sekundo
        self.timer = self.create_timer(3.0, self.run_step)

        get_json_request = SetFloat.Request()
        future = self.cli_json.call_async(get_json_request)
        future.add_done_callback(self.after_json)

    def after_json(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        #self.get_logger().info(f"Prejeti waypoints: {response.data}")
        self.waypoints = [list(pa.vrednosti) for pa in response.data]# pretvori PointArray[] v navaden 2D Python list
        self.have_waypoints = True

        self.get_logger().info(f"Prejeti waypoints: {self.waypoints}")
            
        
    
        

    def run_step(self):
        if not self.have_waypoints or len(self.waypoints) == 0:
            self.get_logger().info("Waypoints še niso naloženi.")
            return

        if self.index >= len(self.waypoints):
            self.get_logger().info('Trajektorija je zaključena.')
            self.timer.cancel()
            return


        get_request = GetAngle.Request()
        future = self.cli_get.call_async(get_request)
        
       
        future.add_done_callback(self.after_get)


    def after_get(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        #self.get_logger().info(f"Trenutna pozicija (robot_return):{response.robot_return},res= {response.res}")
        self.nova_pozicija = response.robot_return
        self.nova_pozicija = self.nova_pozicija.strip('{}')
        self.nova_pozicija = self.nova_pozicija.split(',')
        self.nova_pozicija = [float(x) for x in self.nova_pozicija]
        self.get_logger().info(f'Nova pozicija je: {self.nova_pozicija}')



        if self.index < len(self.waypoints):
            
            #self.waypoints[0] = self.nova_pozicija####deluje dokler obastaja array 
            #self.waypoints.insert(0, self.nova_pozicija)

            goal= self.waypoints[self.index]

            move_request = ServoJ.Request()
            move_request.a, move_request.b, move_request.c, move_request.d, move_request.e, move_request.f = goal


            self.waypoints_np = np.array(self.waypoints)




            razlika = self.waypoints_np[self.index]-self.nova_pozicija #6D vektor razlike
            rezultat = razlika
            max_zasuk = np.max(np.abs(razlika))
            self.get_logger().info(f'Max zasuk je: {razlika}')
            
            cas_rotacije = round(max_zasuk/self.max_hitrost,2)
            self.get_logger().info(f'Čas zasuka je: {cas_rotacije} sekund')

            if cas_rotacije > 3.0:
                self.get_logger().info(f'Prevelik gib za 3s')
                N = math.ceil(cas_rotacije / 3.0)
                self.get_logger().info(f'Prevelik gib za 3s -> razdelim na {N} delov.')

                q_now = np.array(self.nova_pozicija, dtype=float)
                q_goal = np.array(self.waypoints[self.index], dtype=float)

                vmesne_tocke = []
                for k in range(1, N):
                    alpha = k / N
                    q_k = q_now + alpha * (q_goal - q_now)
                    vmesne_tocke.append(q_k.tolist())

                # vstavi vmesne točke
                self.waypoints[self.index:self.index] = vmesne_tocke

                
                goal = self.waypoints[self.index]
                move_request.a, move_request.b, move_request.c, move_request.d, move_request.e, move_request.f = goal
                            
            
                

            #self.get_logger().info(f'Razlika med točkama je: {rezultat}')


            

            # move_request.param_value = rezultat

            move_request.param_value = ['t=3.0']
            #self.get_logger().info(f'vrednost: {move_request.param_value}')

            self.cli_move.call_async(move_request)#tu mu dejansko pošljem ukaz za premik
            #self.get_logger().info(f'Pošiljam pozicijio: {goal}')
            self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = ServoJClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()