// Hollow cylinder with parallel 15° cuts
// All dimensions in millimeters

// Parameters
radius = 60;
diameter = 120;
wall_thickness = 3;
height = 16;           // Height at shortest point
angle = 15;

// Calculate required extra height due to angle
extra_height = diameter * tan(angle);
total_height = height + extra_height;

module block() {
    rotate([angle, 0, 0])
        translate([-diameter, -diameter, -total_height/2])
        cube([diameter * 2, diameter * 2, total_height], center=false);
}

module angledCylinder(height, diameter) {
    difference() {
        cylinder(h = height*3, d = diameter, center=true, $fn = 180);
        //cylinder(h=height*3, r1 = diameter/2*1.05, r2 = diameter/2, center=true, $fn=180)
        
        translate([0, 0, -(height/2 + total_height/2)])
            block();
        translate([0, 0, +(height/2 + total_height/2)])
            block();
    }
}

difference() {
    angledCylinder(height=25, diameter=120);
    translate([0, 0, -4])
        angledCylinder(height=25, diameter=114);
}
