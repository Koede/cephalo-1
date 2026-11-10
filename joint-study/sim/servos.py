"""Bus-servo table, updated from catalog/servo_and_tip_sources.json (maker pages read Nov 8 2026).
Mass, size and stall torque are sourced; price and backlash are estimates (no maker publishes backlash).
Bus-servo table. stall in N*m at the stated voltage. Values are screening values;
see catalog/servo_and_tip_sources.json for the source of each (filled by the research pass).
backlash_deg is an assumption for hobby metal-gear bus servos unless a source says otherwise."""
SERVOS = {
    # model:       mass kg, dims mm (L along arm, W, H), stall Nm, no-load rad/s, price usd, backlash deg
    "STS3215_7V4": dict(mass=0.055, dims=(45.2, 24.7, 35.0), stall=1.91, speed=4.7,  price=17, backlash=0.5),
    "STS3215_12V": dict(mass=0.055, dims=(45.2, 24.7, 35.0), stall=2.94, speed=4.7,  price=19, backlash=0.5),
    "STS3250_12V": dict(mass=0.0745, dims=(45.2, 24.7, 35.0), stall=4.90, speed=4.4,  price=40, backlash=0.5),
    "STS3032":     dict(mass=0.020, dims=(23.2, 12.1, 28.5), stall=0.44, speed=8.7,  price=11, backlash=0.8),
    "SCS0009":     dict(mass=0.014, dims=(23.2, 12.1, 25.3), stall=0.23, speed=10.5, price=8,  backlash=1.0),
    "XL330_M288":  dict(mass=0.018, dims=(34.0, 20.0, 26.0), stall=0.52, speed=10.9, price=24, backlash=0.4),
    "XL430_W250":  dict(mass=0.0572, dims=(46.5, 28.5, 34.0), stall=1.40, speed=6.0,  price=50, backlash=0.25),
    "LX16A":       dict(mass=0.052, dims=(45.2, 24.7, 36.0), stall=1.67, speed=6.0,  price=15, backlash=0.8),
    "HTS35H":      dict(mass=0.064, dims=(54.4, 20.1, 45.5), stall=3.43, speed=4.4,  price=30, backlash=0.6),
    "XL330_M077":  dict(mass=0.018, dims=(34.0, 20.0, 26.0), stall=0.215,speed=40.0, price=24, backlash=0.4),
}
