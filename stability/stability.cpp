#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/actuator_motors.h>

#include <math.h>
#include <string.h>
#include <poll.h>

extern "C" int stability_main(int argc, char *argv[]);

static bool close = false;

float constrain(float val, float min, float max){
        return (val > max) ? max : ((val < min) ? min : val);
}

float modulo(float x){
        if(x >= 0){
                return x;
        }else{
                return x*(-1);
        }

}

int stability_main(int argc, char *argv[]){
                if (!strcmp(argv[1], "stop")){
                        close = true;
                        return 0;
                }
        //SUBSCRIBER SETUP:
                const orb_metadata *id_sen = ORB_ID(sensor_accel);
                int fd = orb_subscribe(id_sen);

                px4_pollfd_struct_t fds[1];
                fds[0].fd = fd;
                fds[0].events = POLLIN;
                struct sensor_accel_s inputs;

        //PUBLISHER SETUP:
                const orb_metadata *id_act = ORB_ID(actuator_motors);
                struct actuator_motors_s outputs;

                orb_advert_t handle = orb_advertise(id_act, &outputs);

        //CONTROLL SETUP:
                uint64_t start_time = hrt_absolute_time();
                double pi = M_PI;
                double kp = 2.0;
                int turn = 0;

        //MAIN LOOP:
                while(!close){
                        turn++;
                        //if(hrt_absolute_time() - start_time >= 10000000){
                        //      PX4_INFO("O tempo acabou!");
                        //      break;
                        //}
                        memset(&outputs, 0, sizeof(outputs));
                        int poll_ret = px4_poll(fds, 1, 1000);
                        if(poll_ret > 0){
                                if(fds[0].revents & POLLIN){
                                        orb_copy(id_sen, fd, &inputs);
                                        double roll = (double)atan2(inputs.y, inputs.z);
                                        double degrees = (double)roll*180/pi;

                                        double sign = (0 - roll)*kp;

                                        sign = (double)constrain((float)sign, -1.0f, 1.0f);

                                        if(turn % 250 == 1){
                                                PX4_INFO("À Inclinação de %.2f graus, o sinal enviado é de: %.2f", degrees, sign);
                                        }

                                        if(modulo(degrees) < 135.0f){
                                                PX4_WARN("Kill Switch ativado!");
                                                outputs.control[3] = 0.0f;
                                        }else{
                                                outputs.control[3] = 0.5f;
                                        }

                                        outputs.timestamp = hrt_absolute_time();
                                        outputs.control[0] = sign;
                                        outputs.control[1] = 0.0f;
                                        outputs.control[2] = 0.0f;
                                        orb_publish(id_act, handle, &outputs);
                                }

                        }else{
                                PX4_ERR("O valor não foi encontrado!");
                        }
                }
        orb_unsubscribe(fd);
        return 0;
}
