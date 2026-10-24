/*
 Smart Pregnancy Belt — CAD model built from the "Design Overview" reference board.
 Units: mm.  Concept / visualization model — NOT a clinically validated medical device.

 What it models (from the reference):
  - 6-layer construction: 1 outer fabric, 2 3D spacer mesh, 3 FSR sensor layer (8 sensors),
    4 memory foam (optional), 5 knitted elastic, 6 inner lining
  - Abdominal support panel (front, taller) with 8 FSR sensors, 2 per quadrant (Q1..Q4)
  - Wide adjustable side straps with hook-and-loop (Velcro) tabs
  - Electronics module under a soft silicone cover (ESP32, LiPo, 8:1 mux, IMU, coin vibration motor)

 Render modes (set with -D 'mode="..."' or edit below):
   "assembled"  – the belt as worn
   "exploded"   – front panel layers pulled apart radially (like the layered cross-section)
 xray = true  – makes the outer cover see-through so sensors/electronics are visible
*/

mode  = "assembled";   // "assembled" | "exploded"
xray  = false;
show_labels = false;   // FSR numbers on the sensors (best with xray=true)
show_body   = false;   // translucent belly reference
strap_open_deg = 0;    // 0 = strap closed; ~60 = right strap peeled open (shows where it opens)
annotate = false;      // callout labels for the strapping system

// ---------- Size ----------
inner_rx = 185;        // inner half-width  (inner perimeter ~ 1.04 m)
inner_ry = 145;        // inner half-depth
back_h   = 105;        // belt height at the back
front_h  = 195;        // belt height at the front panel
RX = inner_rx;  RY = inner_ry;

// ---------- Layer radial offsets from the skin side (mm) ----------
L_lining  = [0.0, 1.2];
L_elastic = [1.15, 3.75];
L_foam    = [3.65, 8.75];
L_film    = [8.7, 9.5];     // FSR sensor layer (flex film + traces)
L_sensor  = [9.45, 10.2];    // sensor discs sit on the film
L_spacer  = [10.15, 13.25];
L_cover   = [13.2, 14.7];   // outer fabric over the panel
L_base    = [3.65, 5.2];     // outer fabric on the rest of the belt
L_strap   = [5.15, 7.2];
L_velcro  = [7.2, 8.0];

panel_t = [235, 305];       // angular extent of the abdominal panel (270 = front centre)
explode_gap = 34;
$fn = 40;

// ---------- Colours ----------
c_outer  = [0.72, 0.62, 0.68];
c_strap  = [0.66, 0.56, 0.63];
c_velcro = [0.45, 0.38, 0.45];
c_lining = [0.96, 0.82, 0.84];
c_elastic= [0.70, 0.68, 0.72];
c_foam   = [0.94, 0.89, 0.80];
c_spacer = [0.60, 0.62, 0.66];
c_film   = [0.20, 0.24, 0.28];
c_fsr    = [0.22, 0.74, 0.46];
c_pcb    = [0.12, 0.45, 0.30];
c_sil    = [0.50, 0.43, 0.50];
c_strap2 = [0.86, 0.79, 0.87];   // lighter webbing so straps read clearly
c_hook   = [0.22, 0.19, 0.24];   // hook (rough) Velcro
c_loop   = [0.95, 0.92, 0.84];   // loop (soft) Velcro
c_tab    = [0.90, 0.50, 0.58];   // pull tab

// ---------- Geometry helpers ----------
function prof(t) = back_h + (front_h - back_h) * pow((1 + cos(t - 270)) / 2, 2);

function ept(t, off) =
    let(nx = cos(t)/RX, ny = sin(t)/RY, n = sqrt(nx*nx + ny*ny))
    [RX*cos(t) + off*nx/n, RY*sin(t) + off*ny/n];

function quad(a,b,c,d) = [[a,b,c],[a,c,d]];
function seg(b,j) = [ each quad(b+0,b+1,j+1,j+0),   // bottom
                      each quad(b+3,j+3,j+2,b+2),   // top
                      each quad(b+1,b+2,j+2,j+1),   // outer face
                      each quad(b+0,j+0,j+3,b+3) ]; // inner face

// Curved band following the belt ellipse from angle t0..t1, between radial offsets o0..o1.
// height = profile(t) - hsub, or constant hc if hc>0. closed=true makes a full ring.
module band(t0, t1, o0, o1, hsub=0, hc=0, zc=0, step=2, closed=false) {
    n  = ceil(abs(t1 - t0)/step);
    ns = closed ? n : n + 1;
    pts = [ for (i=[0:ns-1])
        let(t = t0 + (t1 - t0)*i/n,
            h = (hc > 0 ? hc : prof(t) - hsub),
            a = ept(t, o0), b = ept(t, o1))
        each [[a[0],a[1],zc-h/2],[b[0],b[1],zc-h/2],[b[0],b[1],zc+h/2],[a[0],a[1],zc+h/2]] ];
    last = 4*(ns-1);
    faces = closed
        ? [ for (i=[0:ns-1]) each seg(4*i, 4*((i+1)%ns)) ]
        : concat([ for (i=[0:ns-2]) each seg(4*i, 4*(i+1)) ],
                 [[3,2,1],[3,1,0]], [[last,last+1,last+2],[last,last+2,last+3]]);
    polyhedron(pts, faces, convexity=6);
}

// Local frame on the belt: +X tangent, +Y radially outward (true normal), +Z up.
module on_belt(t, off=0, z=0) {
    nx = cos(t)/RX; ny = sin(t)/RY; n = sqrt(nx*nx + ny*ny);
    translate([RX*cos(t) + off*nx/n, RY*sin(t) + off*ny/n, z])
        rotate([0, 0, atan2(ny, nx) - 90]) children();
}

module rbox(size, r=2) {
    hull() for (sx=[-1,1], sy=[-1,1], sz=[-1,1])
        translate([sx*(size[0]/2 - r), sy*(size[1]/2 - r), sz*(size[2]/2 - r)]) sphere(r=r, $fn=14);
}

// ---------- Sensors (2 per quadrant, 2x2 grid on the front panel) ----------
sensor_x = [-64, -36, 36, 64];       // tangential position (mm from front centre)
sensor_z = [ 36, -36];               // upper row, lower row
function t_of_x(x) = 270 + asin(x / RX);
function sensor_list() = [ for (r=[0:1]) for (c=[0:3]) [t_of_x(sensor_x[c]), sensor_z[r], r*4 + c + 1] ];
// numbering: Q1 (FSR1,2) upper-left, Q2 (FSR3,4) upper-right, Q3 (5,6) lower-left, Q4 (7,8) lower-right

function eo(i) = (mode == "exploded") ? i*explode_gap : 0;   // exploded offset for layer index i

module fsr_sensor(t, z, id) {
    on_belt(t, L_sensor[0] + eo(3), z) {
        translate([0, 0.375, 0]) rotate([90,0,0]) color(c_fsr) cylinder(d=18, h=L_sensor[1]-L_sensor[0], center=true);
        if (show_labels)
            translate([0, 0.4, 0]) rotate([90,0,180]) color("white")
                linear_extrude(0.5) text(str(id), size=6, halign="center", valign="center");
    }
}

module fsr_layer() {
    // flex film
    color(c_film) band(panel_t[0]+3, panel_t[1]-3, L_film[0]+eo(3), L_film[1]+eo(3), hsub=14);
    // traces: tail from each sensor to a bus above/below the grid, buses run to the panel edge
    color([0.85,0.75,0.35]) {
        for (s = sensor_list()) {
            sgn = s[1] > 0 ? 1 : -1;
            on_belt(s[0], L_film[1] + 0.2 + eo(3), sgn*49) cube([1.6, 0.5, 26], center=true);
        }
        for (zb=[62, -62])
            band(t_of_x(-64), panel_t[1]-3, L_film[1]-0.05+eo(3), L_film[1]+0.45+eo(3), hc=1.6, zc=zb);
    }
    for (s = sensor_list()) fsr_sensor(s[0], s[1], s[2]);
}

// ---------- Electronics module (under soft cover, on the right strap) ----------
module electronics_module() {
    base = L_strap[1] - 0.05;                       // sits on the strap
    on_belt(327, 0, -18) {
        // silicone cover: open at the bottom, wall 2.5 mm
        color(c_sil, xray ? 0.30 : 1)
        difference() {
            translate([0, base + 7, 0]) rbox([62, 14, 40], 4);
            translate([0, base + 5.5, 0]) cube([56, 13, 34], center=true);
        }
        // LiPo 3.7 V
        translate([0, base + 2.5, 0])  color([0.25,0.35,0.55]) cube([50, 5, 32], center=true);
        // ESP32 dev board
        translate([0, base + 7.2, 0])  color(c_pcb) cube([48, 4, 26], center=true);
        // 8:1 multiplexer
        translate([-16, base + 10.1, 9]) color([0.1,0.35,0.55]) cube([16, 2, 12], center=true);
        // IMU (6-axis)
        translate([ 16, base + 10.1, 9]) color([0.55,0.15,0.20]) cube([16, 2, 12], center=true);
        // coin vibration motor
        translate([0, base + 10.2, -9]) rotate([90,0,0]) color([0.2,0.2,0.22]) cylinder(d=10, h=3, center=true);
    }
}

// ---------- Straps / closure ----------
// Both wide side straps are sewn to the panel edges (anchor blocks). The LEFT strap lies on the
// panel and carries a loop patch. The RIGHT strap crosses over it and closes with a hook patch;
// a pull tab at its end is where the belt opens.
strap_h  = 80;
LT_left  = [14.65, 16.9];    // left strap on the panel
LP_loop  = [16.85, 17.45];   // loop patch on left strap
LH_hook  = [17.4, 17.95];    // hook patch under right strap
LT_right = [17.9, 20.0];     // right strap (on top)
anchor_R = 306;              // right strap anchor angle (panel right edge)
pivot_pt = ept(anchor_R, 19);

module right_strap_moving() {   // everything that peels away when the belt is opened
    translate([pivot_pt[0], pivot_pt[1], 0]) rotate([0, 0, strap_open_deg])
        translate([-pivot_pt[0], -pivot_pt[1], 0]) children();
}

module straps() {
    sa = xray ? 0.35 : 1;
    // side wings (elastic webbing from the back/sides into the panel)
    color(c_strap2) {
        band(190, 236, L_strap[0], L_strap[1], hc=strap_h);
        band(304, 350, L_strap[0], L_strap[1], hc=strap_h);
    }
    // anchor / stitch blocks where each wing becomes a strap
    color(c_strap2) {
        band(232, 238, L_strap[0], LT_left[1],  hc=strap_h);
        band(302, anchor_R+1, L_strap[0], LT_right[1], hc=strap_h);
    }
    // left strap on the panel + loop patch
    color([0.78,0.71,0.83], sa) band(235, 284, LT_left[0], LT_left[1], hc=strap_h);
    color(c_loop,   sa) band(262, 282, LP_loop[0], LP_loop[1], hc=strap_h - 14);
    // right strap (moves when opened)
    right_strap_moving() {
        color([0.95,0.86,0.92], sa) band(258, anchor_R, LT_right[0], LT_right[1], hc=strap_h);
        color(c_hook,   sa) band(260, 283, LH_hook[0], LH_hook[1], hc=strap_h - 14);
        color(c_tab)        band(248, 258.5, LT_right[0], LT_right[1], hc=46);
    }
}

// ---------- Callouts ----------
function rotp(p) = let(a = strap_open_deg, d = p - pivot_pt)
    [pivot_pt[0] + d[0]*cos(a) - d[1]*sin(a), pivot_pt[1] + d[0]*sin(a) + d[1]*cos(a)];
function wp(t, off, z) = let(q = ept(t, off)) [q[0], q[1], z];

module callout(target, labelpos, txt, size=9) {
    color([0.1,0.1,0.1]) {
        hull() { translate(target) sphere(r=1.6, $fn=10); translate(labelpos) sphere(r=1.0, $fn=10); }
        translate(target) sphere(r=3, $fn=12);
        translate(labelpos + [0, 0, 4]) rotate([90,0,0]) linear_extrude(1.2)
            text(txt, size=size, halign="center", valign="bottom");
    }
}

module annotations() {
    tab_t = strap_open_deg > 0 ? 253 : 253;
    tabp  = ept(tab_t, LT_right[1]);
    hk    = ept(272, LH_hook[1]);
    if (strap_open_deg == 0) {
        callout(wp(253, LT_right[1], 20), [-150, -260, 130], "PULL TAB - belt opens here");
        callout(wp(272, LT_right[1], 48), [ 40, -260, 160], "Right strap (Velcro hook underneath)");
        callout(wp(236, LT_left[1], -30), [-230, -260, -130], "Left strap sewn to panel edge");
        callout(wp(306, LT_right[1], -30), [ 190, -260, -130], "Right strap anchored to panel edge");
    } else {
        rp = rotp(ept(272, LH_hook[1]));
        callout([rp[0], rp[1], -20],        [rp[0]-20, rp[1]-70, -125], "HOOK Velcro (underside)");
        callout(wp(272, LP_loop[1], 10),   [-150, -260, 160], "LOOP patch on left strap");
        callout(wp(anchor_R, LT_right[1], 30), [170, -260, 170], "Hinge: strap peels from here");
        callout(wp(235, LT_left[1], -30),  [-230, -260, -130], "Left strap sewn to panel edge");
    }
}

// ---------- Assemblies ----------
module assembled() {
    ring0 = 90;
    color(c_lining,  1) band(ring0, ring0+360, L_lining[0],  L_lining[1],  hsub=4, closed=true);
    color(c_elastic, 1) band(ring0, ring0+360, L_elastic[0], L_elastic[1], hsub=2, closed=true);
    // outer fabric outside the panel
    color(c_outer) band(panel_t[1]-1, panel_t[0]+361, L_base[0], L_base[1], hsub=0);
    // panel stack (layers 4,3,2,1)
    color(c_foam)   band(panel_t[0]+1.9, panel_t[1]-1.9, L_foam[0],   L_foam[1],   hsub=14);
    fsr_layer();
    color(c_spacer, xray ? 0.55 : 1) band(panel_t[0]+1.9, panel_t[1]-1.9, L_spacer[0], L_spacer[1], hsub=14);
    color(c_outer,  xray ? 0.22 : 1) {
        band(panel_t[0],   panel_t[1],   L_cover[0], L_cover[1], hsub=2);
        band(panel_t[0],   panel_t[0]+2, L_base[1]-0.05,  L_cover[0]+0.05, hsub=2);   // edge walls
        band(panel_t[1]-2, panel_t[1],   L_base[1]-0.05,  L_cover[0]+0.05, hsub=2);
    }
    straps();
    electronics_module();
    if (annotate) annotations();
    if (show_body) color([0.90,0.73,0.63,0.25]) scale([RX-1, RY-1, 120]) sphere(r=1, $fn=64);
}

module exploded() {
    a0 = panel_t[0] - 5;  a1 = panel_t[1] + 5;
    color(c_lining,  1) band(a0, a1, L_lining[0]+eo(0),  L_lining[1]+eo(0),  hsub=4);
    color(c_elastic, 1) band(a0, a1, L_elastic[0]+eo(1), L_elastic[1]+eo(1), hsub=4);
    color(c_foam)       band(a0, a1, L_foam[0]+eo(2),    L_foam[1]+eo(2),    hsub=4);
    fsr_layer();
    color(c_spacer, xray ? 0.35 : 1) band(a0, a1, L_spacer[0]+eo(4),  L_spacer[1]+eo(4),  hsub=4);
    color(c_outer, xray ? 0.18 : 1) band(a0, a1, L_cover[0]+eo(5),   L_cover[1]+eo(5),   hsub=4);
}

if (mode == "exploded") exploded(); else assembled();
