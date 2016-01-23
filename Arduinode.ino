
int internal_pwm_out = 0;
int dbg_count = 0;

void pwm_update( const std_msgs::Int32& pwm_input ){
    internal_pwm_out = (pwm_input.data >> 8) + 128;

    pwm_status_msg.data = internal_pwm_out;
    pwm_status_pub.publish( &pwm_status_msg );
    return;
}

ros::Subscriber<std_msgs::Int32> pwm_input_sub("pwm_signal_input", &pwm_update );

void setup(){
    nh.initNode();
    nh.advertise(pwm_status_pub);
    nh.advertise(arduino_dbg_pub);

    nh.subscribe(pwm_input_sub);
}

void loop(){
    nh.spinOnce();
    dbg_count++;
    nh.spinOnce();
    String dbg_msg = "pwm updated after " + String(dbg_count) + " cycles";
    nh.spinOnce();
    arduino_dbg_msg.data = dbg_msg.c_str();
    nh.spinOnce();
    arduino_dbg_pub.publish( &arduino_dbg_msg );
    nh.spinOnce();
    delay(1000/60);
    analogWrite(11, internal_pwm_out);
}