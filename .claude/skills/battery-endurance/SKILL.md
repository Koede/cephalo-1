---
name: battery-endurance
description: Hover endurance and current-limit check for cephalo-1 packs using battery.py - energy, hover draw, climb current vs connector rating, what-if on pack capacity. Use when choosing the battery.
---
# battery-endurance

Run `python3 joint-study/sim/battery.py` (read its header for flags) and the pack table in DESIGN.md "Battery". Report per pack: mass, energy (Wh), current rating (A), hover draw, hover minutes (energy / (draw * voltage) with 80% usable), rated T/W. Known: any pack added on 15.08 kg drops rated T/W below 1.15; 16,000 mAh 15C gives only 240 A; climb at rated thrust draws ~205 A vs 180 A for two XT90s. Say which numbers are `[A]` assumptions.
