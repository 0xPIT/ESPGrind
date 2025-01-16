
dia = 109.5; // sctually slightly (1.5mm) too small

r = dia /  2;
offset = 17;
lcd_top_border = 9;

// waveshare 2.8 lcd mounted from below
// ordered 2025-01-14
//lcd_l = 60;
//lcd_w = 44;

// CYD capacitive lcd
// ordered 2025-01-14
lcd_l = 50;
lcd_w = 38;

lcd_offset = r - offset - lcd_w / 2 - lcd_top_border;

translate([r, r, 0])
difference() {
    intersection() {
        circle(d=dia, $fn=10000);
        translate([0, -offset, 0])
            square([dia, dia], center=true);
    }
    translate([0, lcd_offset, 0])
        square([lcd_l, lcd_w], center=true);
}

