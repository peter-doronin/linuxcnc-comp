#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_H
#define hal_u32_t uint32_t
#define hal_s32_t int32_t
#define hal_bit_t bool
#define hal_float_t double
#endif

//6040
#define O_SW_ON 0x1
#define O_EN_VOL 0x2
#define O_QSTOP 0x4
#define O_ENOP 0x8
#define O_MODE1 0x10
#define O_MODE2 0x20
#define O_MODE3 0x40
#define O_FAULT_RES 0x80
#define O_HALT 0x100
//6040/

//6060/
#define RESERVED 0
#define PP_MODE 1
#define PV_MODE 3
#define PT_MODE 4
#define HOME_MODE 6
#define IP_MODE 7
#define CSP_MODE 8
#define CSV_MODE 9
#define CST_MODE 10
//6060/


typedef struct{ //6040 reg bit field
    bool SW_ON  : 1;    // 0
    bool EN_VOL : 1;    // 1
    bool QSTOP  : 1;    // 2
    bool ENOP : 1;      // 3
    bool MODE1 : 1;     // 4
    bool MODE2 : 1;     // 5
    bool MODE3 : 1;     // 6
    bool FAULT_RES : 1; // 7
    bool HALT : 1;      // 8
} bit_s_6040;   //not used


typedef struct{ //6041 reg bit field
    bool SW_ON_RDY  : 1;    // 0
    bool SW_ON : 1;         // 1
    bool OP_EN  : 1;        // 2
    bool FAULT : 1;         // 3
    bool VOLT_EN : 1;       // 4
    bool QSTOPED : 1;       // 5
    bool SW_ON_DIS : 1;     // 6
    bool WARN : 1;          // 7
    bool BIT8 : 1;          // 8
    bool REMOTE : 1;        // 9
    bool TARG_REACH : 1;    // 10
    bool INT_LIM : 1;       // 11
    bool MODE1 : 1;         // 12
    bool MODE2 : 1;         // 13
} bit_s_6041;

typedef struct{
    hal_u32_t *out6040;    //control reg OUT
    hal_u32_t  *in6041;    //status reg IN
    hal_u32_t *out6060;    //mode reg OUT
    hal_bit_t *online;
    hal_bit_t *vel_mode;
    hal_bit_t vel_mode_lst;
    hal_bit_t **estop_in;
    hal_bit_t **enable_in;
    hal_bit_t *home_req;
    hal_bit_t *homed_out;
    hal_bit_t *homing_out;
    hal_bit_t *home_err;
    hal_s32_t  *state_out;
    hal_bit_t home_req_lst;
    hal_bit_t enable_in_lst;
    int32_t state;
    uint32_t home_state;
    uint32_t mode;
    hal_bit_t ready;
    hal_bit_t online_lst;
} servio_s;

typedef struct {
    hal_s32_t *out60c1_1;  //interpolate position OUT
    hal_s32_t *out60c1_2;  //interpolete velocity OUT
    hal_s32_t *in6064;     //actual position feedback IN
    hal_float_t *pos_cmd;       //command position IN
    hal_float_t *pos_fb;        //feedback position OUT
    hal_float_t pos_scale;   //position scale parameter
    hal_float_t *offset;    //halpin
    hal_s32_t *debug;       //halpin
    volatile int32_t pos_old1;
    volatile int32_t pos_old2;
    volatile long pos_cmd_long;
    volatile bool offset_reset_last;
    volatile uint32_t in6064_last;
    volatile int32_t highWordFB;
} traj_s;

typedef struct
{
    servio_s IO;
    traj_s TRAJ;
    volatile bool offset_reset;
} servo_data_t;

void calc_command(servo_data_t * sd, int num_chan)
{
    traj_s *tr = &sd[num_chan].TRAJ;
    *tr->debug = num_chan + 10;
    tr->pos_cmd_long = *tr->pos_cmd * tr->pos_scale;
    int32_t highWordCMD = tr->pos_cmd_long >> 32;
    uint32_t lowWordCMD = tr->pos_cmd_long;
    *tr->out60c1_1 = lowWordCMD;
}

void calc_feedback(servo_data_t * sd, int num_chan)
{
    traj_s *tr = &sd[num_chan].TRAJ;
    
    if(((uint32_t)*tr->in6064 - (long)tr->in6064_last) > 100000000L) {tr->highWordFB--;};
    if(((uint32_t)*tr->in6064 - (long)tr->in6064_last) < -100000000L) {tr->highWordFB++;};
    tr->in6064_last = (uint32_t)*tr->in6064;
    
    long pos_fb_long = (uint32_t)*tr->in6064;
    pos_fb_long += ((long)tr->highWordFB << 32);
    *tr->pos_fb = pos_fb_long / tr->pos_scale;    
}

void state_mach(servo_data_t * sd, int num_chan)
{
    //var assignment
    servio_s *sio = &sd[num_chan].IO;
    bit_s_6041 *in;
    bit_s_6040 *out;
    in = (bit_s_6041 *)sio->in6041;
    out = (bit_s_6040 *)sio->out6040;
    hal_bit_t enable_in = **sio->enable_in;
    hal_bit_t estop_in = **sio->estop_in;
    *sio->state_out = sio->state;
    //var assignment/

    if(*sio->home_req != sio->home_req_lst) // home start positive trig
        {
            sio->home_req_lst = *sio->home_req;
            if(*sio->home_req & (!enable_in) & sio->ready)
                {
                    sio->state = 2;
                    sio->mode = HOME_MODE;
                };
        };
    
    if(!estop_in) {sio->state = 3; sio->enable_in_lst = 0;};               //Estop release reaction
    if(!*sio->online){sio->state = 0;};
    if(in->FAULT & (sio->state > 2)) {sio->state = 1;};                           //fault reaction
    
    switch(sio->state) //pre op state
        {
        case 0:                                                                         //offline
            sio->ready = 0;
            sio->home_state = 0;
            *sio->homing_out = 0;
            *sio->out6040 = 0;
            if(*sio->online) {sio->state = 1;};
            if(!sio->online_lst) {sio->online_lst = 1; rtapi_print_msg(RTAPI_MSG_ERR, "SERVO: FAULT (servo-%d offline)\n", num_chan);};
            break;

        case 1:                                                                         //fault
            sio->online_lst = 0;
            sio->ready = 0;
            sio->home_state = 0;
            *sio->homing_out = 0;
            *sio->out6040 = 0;
            if(!estop_in) {sio->state = 2;};
            break;

        case 2:                                                                         //init or fault reset
            sio->ready = 0;
            sio->home_state = 0;
            *sio->homing_out = 0;
            *sio->out6040 = 0;
            if(estop_in)
                {
                    if(in->FAULT | in->WARN)
                        {*sio->out6040 = O_FAULT_RES;}
                    else
                        {*sio->out6040 = 0; sio->state++;};
                };
            break;

        case 3:                                                                         //sw on disabled or warn reset
            if(estop_in)
                {*sio->out6060 = sio->mode; sio->state++;}
            else
                {sio->ready = 0; sio->home_state = 0; *sio->out6040 = 0;};
            break;

        case 4:                                                                         //enable voltage
            if(in->VOLT_EN) {*sio->out6040 = O_EN_VOL | O_QSTOP; sio->state++;};
            break;

        case 5:                                                                         //switch on
            if(in->SW_ON_RDY)
                {*sio->out6040 = O_SW_ON | O_EN_VOL | O_QSTOP; sio->state = 6;};
            break;

        case 6:                                                                         //switched on
            if(in->SW_ON) {sio->state = 7; sio->ready = 1;};
            break;

        case 7:                                                                         //switched on latch
            if(!in->SW_ON) {sio->ready = 0; sio->state = 1;};
            break;

        }               //pre op state/
    

    if(sio->ready) //ready
        {
            switch(sio->mode)
                {
                case 0: if(*sio->vel_mode) {sio->mode = PV_MODE;} else {sio->mode = IP_MODE;}; sio->state = 2; break;
                case HOME_MODE:
                    if(!*sio->home_req) {sio->home_state = 2;};
                    if(enable_in) {sio->home_state = 2; *sio->homed_out = 0;};
                    switch(sio->home_state)
                        {
                        case 0:                                                         //home begin
                            *sio->out6040 = O_SW_ON | O_EN_VOL | O_QSTOP | O_ENOP | O_MODE1;
                            *sio->homing_out = 1;
                            *sio->homed_out = 0;
                            *sio->home_err = 0;
                            sio->home_state = 1;
                            break;

                        case 1:                                                         //waiting servo
                            if(in->MODE1) {sio->home_state = 2; *sio->homed_out = 1;};
                            if(in->MODE2) {sio->home_state = 2; *sio->homed_out = 0; *sio->home_err = 1;};
                            break;
                        case 2:                                                         //mode back change
                            sio->state = 2; sio->home_state = 0;
                            if(*sio->vel_mode) {sio->mode = PV_MODE;} else {sio->mode = IP_MODE;};
                            break;
                        };
                    break;

                case IP_MODE:
                    if(sio->enable_in_lst != enable_in)
                        {
                            sio->enable_in_lst = enable_in;
                            if(enable_in) {*sio->out6040 = O_SW_ON | O_EN_VOL | O_QSTOP | O_ENOP;} else {*sio->out6040 = O_SW_ON | O_EN_VOL | O_ENOP;}; // quick stop or OP
                        };
                    break;
                case PV_MODE:
                    if(sio->enable_in_lst != enable_in)
                        {
                            sio->enable_in_lst = enable_in;
                            if(enable_in) {*sio->out6040 = O_SW_ON | O_EN_VOL | O_QSTOP | O_ENOP;} else {*sio->out6040 = O_SW_ON | O_EN_VOL | O_ENOP;}; // quick stop or OP
                        };
                    break;
                };
        };              //ready/
}

#undef hal_u32_t
#undef hal_s32_t
#undef hal_bit_t
#undef hal_float_t
