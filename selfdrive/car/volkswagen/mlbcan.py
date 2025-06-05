def create_steering_control(packer, bus, apply_angle, PLA_status, PLA_ESP_status, LH_3_Sign):
  values = {
    "PLA_Status_PLA_EPS": PLA_status,
    "PLA_LW_Soll": int(abs(apply_angle) * 10),
    "PLA_LW_Soll_sign": (1 if apply_angle < 0 else 0) if PLA_status == 6 else LH_3_Sign,
    "PLA_Status_PLA_ESP": PLA_ESP_status,
    "PLA_Bremsmoment": 0,
    "PLA_void": 0,
  }

  return packer.make_can_msg("PLA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
  values = ldw_stock_values.copy()

  values.update({
    "LDW_Status_LED_gelb": 1 if enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = gra_stock_values.copy()

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("LS_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  return 0


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  return 0


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  values = {}
  return packer.make_can_msg("ACC_05", bus, values)


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  values = {}
  return packer.make_can_msg("ACC_02", bus, values)
