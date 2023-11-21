/* Autogenerated by /usr/bin/halcompile on Mon Nov 21 12:52:36 2022 -- do not edit */
#include "rtapi.h"
#ifdef RTAPI
#include "rtapi_app.h"
#endif
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "hal.h"
#include "rtapi_math64.h"

static int comp_id;

#ifdef MODULE_INFO
MODULE_INFO(linuxcnc, "component:joint_index:");
MODULE_INFO(linuxcnc, "pin:cmd-in:float:0:in::None:None");
MODULE_INFO(linuxcnc, "pin:sg-cmd:float:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:sg-fb:float:0:in::None:None");
MODULE_INFO(linuxcnc, "pin:fb-in:float:0:in::None:None");
MODULE_INFO(linuxcnc, "pin:fb-out:float:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:enable:bit:0:in::None:None");
MODULE_INFO(linuxcnc, "pin:index-enable:bit:0:io::None:None");
MODULE_INFO(linuxcnc, "pin:cmd-offset:float:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:fb-vel:float:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:fb-accel:float:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:delay-time:float:0:in::None:None");
MODULE_INFO(linuxcnc, "pin:error:float:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:target-error:bit:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:calc-error:bit:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:error-calculated:float:0:out::None:None");
MODULE_INFO(linuxcnc, "pin:debug:bit:0:out::None:None");
MODULE_INFO(linuxcnc, "funct:command:1:");
MODULE_INFO(linuxcnc, "funct:feedback:1:");
MODULE_INFO(linuxcnc, "license:GPL");
MODULE_LICENSE("GPL");
#endif // MODULE_INFO


struct __comp_state {
    struct __comp_state *_next;
    hal_float_t *cmd_in;
    hal_float_t *sg_cmd;
    hal_float_t *sg_fb;
    hal_float_t *fb_in;
    hal_float_t *fb_out;
    hal_bit_t *enable;
    hal_bit_t *index_enable;
    hal_float_t *cmd_offset;
    hal_float_t *fb_vel;
    hal_float_t *fb_accel;
    hal_float_t *delay_time;
    hal_float_t *error;
    hal_bit_t *target_error;
    hal_bit_t *calc_error;
    hal_float_t *error_calculated;
    hal_bit_t *debug;
    bool index_enable_last;

    double command_prev;

    double cmd_index_offset;

    bool index_reset;

    double cmd_in_prev;

    double fb;

    double fb_prev;

    double fb_vel_prev;

    double delay_offset;

    double error_offset;

    int calc_error_timer;

};
struct __comp_state *__comp_first_inst=0, *__comp_last_inst=0;

static void command(struct __comp_state *__comp_inst, long period);
static void feedback(struct __comp_state *__comp_inst, long period);
static int __comp_get_data_size(void);
#undef TRUE
#define TRUE (1)
#undef FALSE
#define FALSE (0)
#undef true
#define true (1)
#undef false
#define false (0)

static int export(char *prefix, long extra_arg) {
    char buf[HAL_NAME_LEN + 1];
    int r = 0;
    int sz = sizeof(struct __comp_state) + __comp_get_data_size();
    struct __comp_state *inst = hal_malloc(sz);
    memset(inst, 0, sz);
    r = hal_pin_float_newf(HAL_IN, &(inst->cmd_in), comp_id,
        "%s.cmd-in", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_OUT, &(inst->sg_cmd), comp_id,
        "%s.sg-cmd", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_IN, &(inst->sg_fb), comp_id,
        "%s.sg-fb", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_IN, &(inst->fb_in), comp_id,
        "%s.fb-in", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_OUT, &(inst->fb_out), comp_id,
        "%s.fb-out", prefix);
    if(r != 0) return r;
    r = hal_pin_bit_newf(HAL_IN, &(inst->enable), comp_id,
        "%s.enable", prefix);
    if(r != 0) return r;
    r = hal_pin_bit_newf(HAL_IO, &(inst->index_enable), comp_id,
        "%s.index-enable", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_OUT, &(inst->cmd_offset), comp_id,
        "%s.cmd-offset", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_OUT, &(inst->fb_vel), comp_id,
        "%s.fb-vel", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_OUT, &(inst->fb_accel), comp_id,
        "%s.fb-accel", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_IN, &(inst->delay_time), comp_id,
        "%s.delay-time", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_OUT, &(inst->error), comp_id,
        "%s.error", prefix);
    if(r != 0) return r;
    r = hal_pin_bit_newf(HAL_OUT, &(inst->target_error), comp_id,
        "%s.target-error", prefix);
    if(r != 0) return r;
    r = hal_pin_bit_newf(HAL_OUT, &(inst->calc_error), comp_id,
        "%s.calc-error", prefix);
    if(r != 0) return r;
    r = hal_pin_float_newf(HAL_OUT, &(inst->error_calculated), comp_id,
        "%s.error-calculated", prefix);
    if(r != 0) return r;
    r = hal_pin_bit_newf(HAL_OUT, &(inst->debug), comp_id,
        "%s.debug", prefix);
    if(r != 0) return r;
    rtapi_snprintf(buf, sizeof(buf), "%s.command", prefix);
    r = hal_export_funct(buf, (void(*)(void *inst, long))command, inst, 1, 0, comp_id);
    if(r != 0) return r;
    rtapi_snprintf(buf, sizeof(buf), "%s.feedback", prefix);
    r = hal_export_funct(buf, (void(*)(void *inst, long))feedback, inst, 1, 0, comp_id);
    if(r != 0) return r;
    if(__comp_last_inst) __comp_last_inst->_next = inst;
    __comp_last_inst = inst;
    if(!__comp_first_inst) __comp_first_inst = inst;
    return 0;
}
static int default_count=1, count=0;
RTAPI_MP_INT(count, "number of joint_index");
char *names = ""; // comma separated names
RTAPI_MP_STRING(names, "names of joint_index");
int rtapi_app_main(void) {
    int r = 0;
    int i;
    comp_id = hal_init("joint_index");
    if(comp_id < 0) return comp_id;
    if(count && names[0]) {
        rtapi_print_msg(RTAPI_MSG_ERR,"count= and names= are mutually exclusive\n");
        return -EINVAL;
    }
    if(!count && !names[0]) count = default_count;
    if(count) {
        for(i=0; i<count; i++) {
            char buf[HAL_NAME_LEN + 1];
            rtapi_snprintf(buf, sizeof(buf), "joint-index.%d", i);
            r = export(buf, i);
            if(r != 0) break;
       }
    } else {
        size_t i, j;
        int idx;
        char buf[HAL_NAME_LEN+1];
        const size_t length = strlen(names);
        for (i = j = idx = 0; i <= length; i++) {
            const char c = buf[j] = names[i];
            if ((c == ',') || (c == '\0')) {
                buf[j] = '\0';
                r = export(buf, idx);
                if(r != 0) {break;}
                idx++;
                j = 0;
            } else {
                if (++j == (sizeof(buf) / sizeof(buf[0]))) {
                    buf[j - 1] = '\0';
                    rtapi_print_msg(RTAPI_MSG_ERR,"names: \"%s\" too long\n", buf);
                    r = -EINVAL;
                    break;
                }
            }
        }
    }
    if(r) {
        hal_exit(comp_id);
    } else {
        hal_ready(comp_id);
    }
    return r;
}

void rtapi_app_exit(void) {
    hal_exit(comp_id);
}

#undef FUNCTION
#define FUNCTION(name) static void name(struct __comp_state *__comp_inst, long period)
#undef EXTRA_SETUP
#define EXTRA_SETUP() static int extra_setup(struct __comp_state *__comp_inst, char *prefix, long extra_arg)
#undef EXTRA_CLEANUP
#define EXTRA_CLEANUP() static void extra_cleanup(void)
#undef fperiod
#define fperiod (period * 1e-9)
#undef cmd_in
#define cmd_in (0+*__comp_inst->cmd_in)
#undef sg_cmd
#define sg_cmd (*__comp_inst->sg_cmd)
#undef sg_fb
#define sg_fb (0+*__comp_inst->sg_fb)
#undef fb_in
#define fb_in (0+*__comp_inst->fb_in)
#undef fb_out
#define fb_out (*__comp_inst->fb_out)
#undef enable
#define enable (0+*__comp_inst->enable)
#undef index_enable
#define index_enable (*__comp_inst->index_enable)
#undef cmd_offset
#define cmd_offset (*__comp_inst->cmd_offset)
#undef fb_vel
#define fb_vel (*__comp_inst->fb_vel)
#undef fb_accel
#define fb_accel (*__comp_inst->fb_accel)
#undef delay_time
#define delay_time (0+*__comp_inst->delay_time)
#undef error
#define error (*__comp_inst->error)
#undef target_error
#define target_error (*__comp_inst->target_error)
#undef calc_error
#define calc_error (*__comp_inst->calc_error)
#undef error_calculated
#define error_calculated (*__comp_inst->error_calculated)
#undef debug
#define debug (*__comp_inst->debug)
#undef index_enable_last
#define index_enable_last (__comp_inst->index_enable_last)
#undef command_prev
#define command_prev (__comp_inst->command_prev)
#undef cmd_index_offset
#define cmd_index_offset (__comp_inst->cmd_index_offset)
#undef index_reset
#define index_reset (__comp_inst->index_reset)
#undef cmd_in_prev
#define cmd_in_prev (__comp_inst->cmd_in_prev)
#undef fb
#define fb (__comp_inst->fb)
#undef fb_prev
#define fb_prev (__comp_inst->fb_prev)
#undef fb_vel_prev
#define fb_vel_prev (__comp_inst->fb_vel_prev)
#undef delay_offset
#define delay_offset (__comp_inst->delay_offset)
#undef error_offset
#define error_offset (__comp_inst->error_offset)
#undef calc_error_timer
#define calc_error_timer (__comp_inst->calc_error_timer)


#line 41 "joint_index.comp"

#include "rtapi_math.h"

#define MAX_ERROR_OFFSET 0.1

double filter(double input, double time_in, double f_per);
 
FUNCTION(feedback)
 {
	debug = !debug;

	
    if(index_enable_last != index_enable)
    {
        index_enable_last = index_enable;
        if(!index_enable) 
        {
			index_reset = 1;
			calc_error = 1;
			calc_error_timer = 0;
            fb_prev = 0;
        }
    }		

	
	fb = fb_in;
	
    fb_vel = (fb - fb_prev) / fperiod;
    fb_accel = (fb_vel - fb_vel_prev) / fperiod;
    fb_vel_prev = fb_vel;
    fb_prev = fb;
    delay_offset = fb_vel * delay_time + fb_accel * delay_time * delay_time / 2;
	fb_out = fb + delay_offset;
	
	cmd_in_prev = cmd_in;
	error = cmd_in - fb_out;

 }

FUNCTION(command)
 {
	if(calc_error)
	{
		if(cmd_in == cmd_in_prev)
		{
			calc_error_timer++;
		} else {calc_error_timer = 0;};
		if(calc_error_timer > 200)
		{
			error_calculated = error_offset + error;
			if(fabs(error_calculated) > 0.0005) target_error = 1;
			calc_error = 0;
			calc_error_timer = 0;
		}
	}
	if(target_error && enable)
	{
		if(fabs(error_calculated - error_offset) > 0.0001)
		{
			error_offset += (error_calculated - error_offset) / 1000;
		} else target_error = 0;
	}
	if(error_offset > MAX_ERROR_OFFSET) {error_offset = MAX_ERROR_OFFSET; target_error = 0;};
	if(error_offset < -MAX_ERROR_OFFSET) {error_offset = -MAX_ERROR_OFFSET; target_error = 0;};
	
	if(index_reset)
	{
		index_reset = 0;
		cmd_offset = sg_fb - cmd_in;
	}
	
	if(!enable)
	{
        cmd_offset = sg_fb - cmd_in;
        error_offset = 0;
    }
    else
    {
        sg_cmd = cmd_in + cmd_offset + error_offset;        
    }
 }

double filter(double input, double time_in, double f_per)
	{
		#define RING_SIZE 100
		int time = (int)(fabs(time_in) / f_per); 
		if(time == 0) return input;
		if(time > RING_SIZE) time = RING_SIZE;
		
		static unsigned int ring_index;
		static double ring_buf[RING_SIZE];
		ring_index++;
		ring_index %= time;
		ring_buf[ring_index] = input;
		double f_out;
		for(int i = 0; i < time; i++)
		{
			f_out += ring_buf[i];
		}
		f_out /= time;
		return f_out;
	}



static int __comp_get_data_size(void) { return 0; }
