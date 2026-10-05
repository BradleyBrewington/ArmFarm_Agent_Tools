Read-only temperature diagnosis

No motion, torque enabling, or episode creation during this diagnostic turn. Gripper remains torque-off, arm at home, cube on table, recorder inactive.

First700 paired samples: individual temperature reads47–48C; adjacent voltage+temperature reads47–50C. Extended2556 paired samples: individual reads46–66C; adjacent reads46–50C. Three single-read values60C or higher were each followed by a paired temperature of47C. Motor fault bits and communication result codes were zero.

The reads address the same temperature register63: single-byte read at63 versus two-byte read at62 (voltage byte followed by temperature byte). The SDK decodes single-byte reads directly and combines paired bytes as a little-endian word; no evident decode bug was found. The relay serializes bus transactions and checks packet length/checksum. These inspections do not prove hardware or communications health.

The discrepancy is reproducible at idle and remains unresolved. Do not discard high readings, change the60C guard, or switch read methods solely to avoid the guard. Sensor/controller and serial feedback diagnosis is required before sustained collection resumes.

Evidence: temperature_crosscheck.json and temperature_crosscheck_extended.json. Collection remains51 successful episodes,43 clean recordings; planned grid indices41–44 incomplete.

Latest recheck: 2,054 paired samples while stationary with gripper torque off. Single-register readings reached 62 C; immediately following paired reads were 46 C. Recorder inactive. Fault remains reproducible. Evidence: temperature_recheck.json.

Operator subsequently authorized resumption (message a4b8b931-c9fb-42cd-a0e8-88e8353a0eb8) under an updated65C software guard and unchanged70C hardware protection. Verified both before motion. Earlier pause recommendation is superseded by this operator instruction; root cause remains unconfirmed.
