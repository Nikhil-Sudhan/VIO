# Prompt for a rigid Raspberry Pi visual–inertial enclosure

Use this prompt with a mechanical CAD generator. All dimensions are in millimetres. Manufacturer dimensions are reference geometry; enclosure dimensions and clearances below are proposed design parameters, not measurements of the existing assembly.

Reference drawings:
- Raspberry Pi 5: https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf
- Camera Module 2 reference PCB: https://datasheets.raspberrypi.com/camera/camera-module-2-mechanical-drawing.pdf
- Adafruit LSM6DSOX PCB and fabrication geometry: https://github.com/adafruit/Adafruit-LSM6DSOX-PCB
- Adafruit product dimensions: https://www.adafruit.com/product/4438

BEGIN PROMPT

Design a functional, parametric, 3D-printable enclosure for a monocular visual–inertial odometry (VIO) module running OpenVINS. Produce manufacturable CAD with separate printable parts, accurate component interfaces, and an assembly drawing. The main priority is keeping the camera and IMU rigidly fixed relative to each other during handling and movement.

1. COMPONENTS AND DESIGN BASIS

Include:
- Raspberry Pi 5 Model B, 8 GB. Reference PCB envelope: 85 x 56 mm. Use the manufacturer's actual mounting-hole pattern and connector geometry. Reference mounting-hole spacing is 58 x 49 mm; do not center that rectangle arbitrarily on the PCB.
- Existing aluminium heatsink and blower fan fitted to the Pi, as shown in the reference photos. Preserve both. Make the cooler envelope, fan intake, exhaust and plug clearances explicit parameters; confirm the cooler variant before finalizing its cavity.
- Sony IMX219 CSI camera board. A standard Raspberry Pi Camera Module 2 is approximately 25 x 24 mm, with four 2.2 mm mounting holes. Use its drawing only if the actual camera PCB matches. Otherwise keep the camera outline, hole locations, lens position and connector geometry editable. Sensor identification alone does not establish a third-party board's dimensions.
- Adafruit LSM6DSOX accelerometer/gyroscope breakout, STEMMA QT version, product 4438. Published assembled dimensions are approximately 25.6 x 17.8 x 4.6 mm. The published Eagle PCB outline is 25.4 x 17.78 mm, with two nominal 2.5 mm mounting holes, 20.32 mm apart. Use the PCB revision matching the photographs, and allow extra room for the actual soldered header, jumper plugs and STEMMA QT sockets.
- Existing camera ribbon/adapter cable, four-wire I2C harness, fan lead and external USB-C power connection. Model their connectors and service clearances, not just the bare wires.

The new case replaces the existing plastic Pi enclosure. Start with the Pi PCB and its fitted cooler. Do not make an enclosure around the old plastic case. Do not add a battery, display, flight controller or extra sensors.

2. EXACT SENSOR ORIENTATION

Define a right-handed body frame at the IMU sensing element:
- +X: forward, looking out through the camera lens.
- +Y: left, as seen by a person behind the camera looking forward.
- +Z: upward when the camera image is upright.

Mount the IMU so its actual sensor-axis/silkscreen arrows agree with this frame. Verify against the PCB drawing; do not infer the axes from the connector direction.

Mount the camera PCB vertically, approximately in the body YZ plane, with its lens facing +X. Mount the IMU PCB horizontally, approximately in the body XY plane, with sensor +Z upward. These PCB planes are perpendicular. Both must share the same rigid support structure.

The camera optical axes are: image right = body -Y; image down = body -Z; optical forward = body +X. Add an upright-image orientation mark to prevent upside-down camera installation.

Place the IMU sensing element behind the camera, toward -X. Aim to align the IMU sensing-element center with the camera lens axis in height and sideways position. Use an initial camera-to-IMU forward separation of 30 mm, adjustable in CAD over roughly 20–45 mm to accommodate the real components and ribbon bend. This is a proposed layout dimension, not an existing calibrated measurement. Small fixed height/side offsets are acceptable when necessary; report their actual CAD values.

3. RIGID SENSOR STRUCTURE

Use a single rigid sensor carrier incorporating a vertical camera plate and a horizontal IMU platform. Add substantial triangular side gussets joining them. Do not rely on an unsupported thin L-shaped wall.

Use screw mounting and positive locating surfaces for both boards. Locate the camera using its mounting holes and PCB datums; do not clamp or load the lens barrel. Support the IMU at its mounting holes and provide an edge-registration feature to prevent yaw rotation. Keep supports clear of underside components and solder joints. Do not press against the IMU package.

Both sensors must move as one rigid body. Do not put foam, rubber, springs, a ball joint, a flexible arm or tape between one sensor and the common carrier. If a separate carrier is used, attach it to the main base with a keyed seat, hard stops and four screws; the camera-to-IMU relationship must remain within the one-piece carrier.

Make the outer lid removable without loosening either sensor or pulling its cables. The lid must not determine sensor alignment.

4. ENCLOSURE LAYOUT

Use a front sensor compartment and a rear Pi compartment. Place the Pi horizontally behind the sensor carrier. Prefer the USB/Ethernet end toward the rear when connector and cable clearances permit. Derive all port openings from the oriented Pi CAD.

Use an initial outer-envelope target of about 125 mm front-to-back, 80 mm wide and 45 mm high. Adjust this after checking the real cooler, connectors, cable bends and sensor mounting height. Prioritize clearance and rigidity over forcing this envelope.

Provide a rigid base, screw-fastened ventilated lid, protected front lens opening, and accessible cable guides. Include a flat bottom and reinforced external attachment points. Keep mounting loads in the chassis, not in PCB screws or the lid. Make any external mounting-hole pattern a named CAD parameter.

5. CAMERA OPENING

Provide a lens opening centered on the actual optical axis. Start with a generously sized aperture, then verify clearance using the camera's full field of view and actual lens position. Chamfer the inner opening and keep any protective rim shallow enough to avoid vignetting. Do not recess the lens in a narrow tunnel.

Keep the opening clear of cables, screws and printed ribs. Leave access to the camera connector latch during assembly. Do not add an optical window or lens cover in the imaging path. Include a removable storage cap if useful.

6. CABLE ROUTING AND STRAIN RELIEF

Route the camera ribbon from its connector through a protected channel toward the Pi's camera socket. For a standard Module 2 connected to a Pi 5, account for the correct 15-pin-to-mini-22-pin camera cable/adapter and its two different end widths. Match the actual fitted cable and contact orientation.

Use a broad, smooth bend; begin with a 10 mm routing radius where practical and respect the cable supplier's minimum. Avoid sharp creases, tension, twisting, screw contact and pinching at the lid seam. Provide a small service loop and gentle strain relief at both ends without loading the connector latches.

If the ribbon passes beneath the IMU, create a dedicated underpass below a rigid raised platform. Allow the full ribbon width plus approximately 2 mm lateral clearance on each side, and sufficient vertical clearance for its real bend and clamps. Keep it clear of the IMU's support posts. Do not let the ribbon push on either board.

Route the four-wire IMU harness in a separate side channel. Label VIN, GND, SDA and SCL, preserving the existing electrical connections. Do not use wire colors to infer signals. Add gentle cable clamps and a small slack loop so cable movement cannot rotate the IMU or bend its PCB. Keep wires away from the lens, fan and hot heatsink surfaces.

Provide a short protected route for the fan lead. Include finger/tool access to plugs and latches. External USB-C cable loads must be taken by the enclosure rather than by the sensor carrier.

7. COOLING AND PORT ACCESS

Retain the existing Pi heatsink/fan mounting. Provide an unobstructed intake above the blower and an exhaust path aligned with the cooler. Include a guard that cannot contact the fan. Route sensor wiring around this airflow path.

Keep the IMU away from direct heatsink contact and the hot exhaust where possible. Maintain a rigid sensor support while providing spatial/thermal separation from the Pi; do not suspend the IMU independently on soft material.

Provide correctly positioned openings for USB-C power, both micro-HDMI connectors, USB ports, Ethernet, microSD access and the Pi power button. Allow real cable-plug bodies and finger clearance. Provide service access to the used GPIO pins and camera connector. Avoid metal inserts over the wireless antenna region.

8. PRINTING, FASTENERS AND TOLERANCES

Design for FDM printing in PETG or ASA with a 0.4 mm nozzle and approximately 0.2 mm layers. Treat these as initial print settings, subject to the printer and thermal environment.

Suggested starting dimensions:
- Outer walls: 2.4–3 mm.
- Base: 3–4 mm.
- Sensor carrier and gussets: approximately 4 mm, increased where required.
- Fillets at load-bearing corners: approximately 2–3 mm.
- Non-clamping mating clearance: approximately 0.25–0.35 mm per side, adjustable after a fit test.

Keep sensor locating surfaces and screw seats rigid and repeatable; do not allow the general enclosure clearance to become sensor play. Use proper standoffs and suitable washers. Use M2 hardware where appropriate for the camera and IMU, M2.5 for Pi mounting, and M3 screws/inserts for structural case joints. Check every fastener against the actual board holes and surrounding components. Derive screw lengths from the assembled stack. Size insert holes from the chosen insert manufacturer's dimensions.

Favor screws and heat-set inserts over a snap-fit-only closure. Add screw-head/tool clearance, avoid screws bottoming against electronics, and leave room for underside components. Choose print orientations that strengthen the camera/IMU junction. Keep support material away from optical openings, connector latches and precision seating surfaces.

9. REQUIRED DELIVERABLES

Provide:
- Editable parametric CAD and STEP assembly.
- A separate watertight STL/3MF for each printed part, in millimetres.
- Dimensioned front, side, top and section views.
- An exploded assembly and complete fastener BOM.
- A cable-routing view with both ribbon end orientations and bend envelopes.
- Clearly marked +X FORWARD, +Y LEFT and +Z UP arrows on the sensor carrier.
- CAD datums at the IMU sensing-element center and the nominal camera optical reference, with their fixed relative position and orientation documented. Distinguish a mechanical lens reference from an optically calibrated projection center.
- A table separating manufacturer dimensions, proposed dimensions and unverified dimensions.
- Interference checks for installed boards, solder joints, fan, cables, screw heads, plugs, latches and lid removal.
- A small mounting/clearance test coupon so the critical fits can be checked before printing the full case.

The finished assembly must preserve camera-to-IMU rigidity when picked up, when cables are touched, and when the lid is removed. Final camera–IMU extrinsic calibration must be repeated after mounting the hardware in this case. Do not claim that CAD alignment or a rigid enclosure alone eliminates VIO drift.

END PROMPT
