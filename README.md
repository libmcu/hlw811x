# HLW811x driver

## Usage

```c
struct hlw811x_coeff coeff;

hlw811x_init(HLW811X_UART);
hlw811x_reset();
sleep_ms(60);

hlw811x_read_coeff(&coeff);

hlw811x_set_resistor_ratio(&(const struct hlw811x_resistor_ratio) {
    .K1_A = 1,
    .K1_B = 1,
    .K2 = 1,
});
hlw811x_set_pga(&(const struct hlw811x_pga) {
    .A = HLW811X_PGA_GAIN_2,
    .B = HLW811X_PGA_GAIN_2,
    .U = HLW811X_PGA_GAIN_2,
});
hlw811x_set_channel_b_mode(HLW811X_B_MODE_NORMAL);
hlw811x_set_active_power_calc_mode(HLW811X_ACTIVE_POWER_MODE_POS_NEG_ALGEBRAIC);
hlw811x_set_rms_calc_mode(HLW811X_RMS_MODE_AC);
hlw811x_set_data_update_frequency(HLW811X_DATA_UPDATE_FREQ_HZ_3_4);
hlw811x_set_zerocrossing_mode(HLW811X_ZEROCROSSING_MODE_POSITIVE);
hlw811x_enable_waveform();
hlw811x_enable_zerocrossing();
hlw811x_enable_powerfactor();
hlw811x_enable_channel(HLW811X_CHANNEL_ALL);
hlw811x_enable_pulse(HLW811X_CHANNEL_ALL);

hlw811x_select_channel(HLW811X_CHANNEL_A);

int32_t mV, mA, mW, watt, centiHz, pf_centi, centidegree;

hlw811x_get_rms(HLW811X_CHANNEL_U, &mV);
hlw811x_get_rms(HLW811X_CHANNEL_A, &mA);
hlw811x_get_power(&mW);
hlw811x_get_energy(HLW811X_CHANNEL_A, &watt);
hlw811x_get_frequency(&centiHz);
hlw811x_get_power_factor(&pf_centi);
hlw811x_get_phase_angle(&centidegree, HLW811X_LINE_FREQ_60HZ);
```
