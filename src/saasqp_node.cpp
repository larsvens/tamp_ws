#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#define NX          ACADO_NX	/* number of differential states */
#define NXA         ACADO_NXA	/* number of alg. states */
#define NU          ACADO_NU	/* number of control inputs */
#define N           ACADO_N	/* number of control intervals */
#define NY	    ACADO_NY	/* number of references, nodes 0..N - 1 */
#define NYN	    ACADO_NYN
#define NUM_STEPS   100		/* number of simulation steps */
#define VERBOSE     1		/* show iterations: 1, silent: 0 */

class SAASQP
{
public:
    SAASQP(ros::NodeHandle nh){
        nh_ = nh;
        dt = 0.1;
        ros::Rate loop_rate(1/dt);

        // pubs
        chatter_pub_ = nh_.advertise<std_msgs::String>("chatter", 1000);

        int count = 0;

        // run loop
        while (ros::ok())
        {
          std_msgs::String msg;
          std::stringstream ss;
          ss << "hello world " << count;
          msg.data = ss.str();
          ROS_INFO("%s", msg.data.c_str());
          chatter_pub_.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
          ++count;
        }
    }
private:
    double dt;
    ros::NodeHandle nh_;
    ros::Publisher chatter_pub_;

    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "saasqp");
  ros::NodeHandle nh;
  SAASQP saasqp(nh);
  return 0;
}
