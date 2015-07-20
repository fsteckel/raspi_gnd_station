#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include "VG/openvg.h"
#include "VG/vgu.h"
#include "../fontinfo.h"
#include "../shapes.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/resource.h>

// #include <arpa/inet.h>
// #include <sys/socket.h>
// #include <netinet/in.h>

int BUF, fd_joystick;
int go = 1;

#define MIN(a,b) ((a) < (b) ? (a) : (b))    // Funktion MIN(a,b) definieren
#define MAX(a,b) ((a) < (b) ? (b) : (a))    // Funktion MIN(a,b) definieren

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
	double noconstrain = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	return MAX(MIN(noconstrain,out_max),out_min);
}



#define JS_EVENT_BUTTON         0x01
#define JS_EVENT_AXIS           0x02
#define JS_EVENT_INIT           0x80
#define JS_X			2
#define	JS_Y			0
#define JS_Z			1
#define JS_T			4
#define JS_B0			0
#define JS_B1			1

struct rusage usage;
struct timeval start, end;

typedef struct {
	int16_t eta;
	int16_t phi;
	int16_t xi;
	int16_t thr;
} joy_position;
joy_position joy_pos;


double angleold = 0;

typedef struct {
	Fontinfo font;
	VGfloat tw;
	int fontsize;
} FW;

struct js_event {
	uint32_t time;     /* event timestamp in milliseconds */
	int16_t value;    /* value */
	uint8_t type;      /* event type */
	uint8_t number;    /* axis/button number */
};

typedef struct{
	double mode; // 0 manual 1 automatic
	double segment;
	double dist; // [NM] to next
	double time; //[s] to next
	double action; // 0 überflug 1 rechtskreis -1 linkskreis
	
	double vert_ctl; // 0 none 1 h_dot 2 h 3 gamma 4 gs (5 flare)
	double vert_ist;
	double vert_soll;
	double vert_cmd;
	
	double lat_ctl; // 0 none 1 bank 2 hdg 3 trk 4 loc 5 circle
	double lat_ist; // Kreisflug: Istabstand
	double lat_soll; // Kreisflug: Sollkreisradius
	double lat_cmd;
	double lat_reserve; // Kreisflug = bearing from point to Aircraft; bei den anderen Reglern nicht verwendet, also z.B. = 0
	
	double spd_ctl; // 0 none 1 elev 2 thrust
	double spd_ist;
	double spd_soll;
	double spd_cmd;
	
	double gear_ist;// [%]
	double gear_soll; // 0 up 1 dn
	
	double eng1_pow; // [%]
	double eng2_pow; // [%]
	double eng1_pow_soll; // [%]
	double eng2_pow_soll; // [%]
	
	double flaps_ist;
	double flaps_soll; // 0 17 34
	
	double winddir;
	double windvel;
	
	double reserve1;
	double reserve2;
} statedef;
statedef state;

void stateinit(void) {
	state.mode= 2; // 0 manual 1 automatic
	state.segment= 0;
	state.dist= 0; // [NM] to next
	state.time= 0; //[s] to next
	state.action= 0; // 0 überflug 1 rechtskreis -1 linkskreis
	
	state.vert_ctl = 0; // 0 none 1 h_dot 2 h 3 gamma 4 gs (5 flare)
	state.vert_ist= 0;
	state.vert_soll= 0;
	state.vert_cmd= 0;
	
	state.lat_ctl = 0; // 0 none 1 bank 2 hdg 3 trk 4 loc 5 circle
	state.lat_ist= 0; // Kreisflug: Istabstand
	state.lat_soll= 0; // Kreisflug: Sollkreisradius
	state.lat_cmd= 0;
	state.lat_reserve= 0; // Kreisflug = bearing from point to Aircraft= 321.45; bei den anderen Reglern nicht verwendet, also z.B. = 0
	
	state.spd_ctl = 0; // 0 none 1 elev 2 thrust
	state.spd_ist= 0;
	state.spd_soll= 0;
	state.spd_cmd= 0;
	
	state.gear_ist= 0;// [%]
	state.gear_soll= 0; // 0 up 1 dn
	
	state.eng1_pow= 0; // [%]
	state.eng2_pow= 0; // [%]
	
	state.flaps_ist= 0;
	state.flaps_soll= 0; // 0 17 34
	
	state.winddir= 0;
	state.windvel= 0;
	
	state.reserve1= 0;
	state.reserve2= 0;
}

int send_cmd = 0;

int fontsize;

void coordpoint(VGfloat x, VGfloat y) {     // Koordinatenpunkt markieren (fuer debugging)
	VGfloat size = 7;
	Fill(255, 255, 255, 1);
	Circle(x, y, size);
}

void waituntil(int endchar) {
	int key;
	
	for (;;) {
		key = getchar();
		if (key == endchar || key == '\n') {
			break;
		}
	}
}

void fitwidth(int width, int height, int adj, char *s, FW * f) {    // Text in HÃ¶he und Breite einpassen
	f->fontsize = MIN(height, f->fontsize);
	f->tw = TextWidth(s, f->font, f->fontsize);
	while ((f->tw > width)) {
		f->fontsize -= adj;
		f->tw = TextWidth(s, f->font, f->fontsize);
	}
}

void pointer(VGfloat x,VGfloat y,VGfloat r,double deg_ist,double deg_cmd,double deg_soll) {
	VGfloat dartangle = 0.5236;
	Translate(x,y);
	VGfloat dartlen = r/8;
	
	Stroke(75,75,75,1); // Linieninitialisierung
	StrokeWidth(r/75);
	Fill(75,75,75,1);
	
	Rotate(-deg_soll);
	Line(0,0,0,r*1.05);
	Rotate(deg_soll);
	Fill(0,0,0,0);
	
	Stroke(255,140,0,1); // Linieninitialisierung
	StrokeWidth(r/50);
	Fill(255,140,0,1);
	
	Rotate(-deg_cmd);
	Line(0,0,0,r);
	Circle(0,r,r/20);
	Rotate(deg_cmd);
	Fill(0,0,0,0);
	
	Stroke(255,255,255,1); // Linieninitialisierung
	StrokeWidth(r/30);
	
	Rotate(-deg_ist);
	r*=0.9;
	Line(0,0,0,r);
	Line(0,r*0.9875,-dartlen*sin(dartangle),r-dartlen*cos(dartangle));
	Line(0,r*0.9875,dartlen*sin(dartangle),r-dartlen*cos(dartangle));
	Rotate(deg_ist);
	
	Fill(255,255,255,1);
	Circle(0,0,r/10);
	
	Translate(-x,-y);
	Stroke(0,0,0,0);
	Fill(0,0,0,0);
}

void indicator(VGfloat x, VGfloat y, int len, int type, char *ind){
	Translate(x,y);
	Stroke(0,0,0,0);
	int fontsize_loc = 1.5*fontsize;
	VGfloat h = 1.3333*fontsize_loc;
	VGfloat w = ((1+len) * fontsize_loc) * 0.85;
	FW tw2 = { SansTypeface, 0, fontsize_loc }; //Schrift initialisieren
	
	Fill(75,75,75,1);
	Roundrect(-w/2,-h/2,w,h,fontsize_loc/2,fontsize_loc/2);
	
	if (type == 0)
		Fill(255,255,255,1);
	if (type == 1)
		Fill(255,140,0,1);
	
	TextEnd(w/2 - tw2.fontsize/2,-tw2.fontsize/2, ind, tw2.font, tw2.fontsize);
	
	Translate(-x,-y);
}

void gsplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double dev, int type){
	char *test = "-1000";
	int length = strlen(test);
	char *text = malloc(length + 1);
// 	int fontsize = 256;
	Fill(0,0,0,1);
	Rect(x+1,y+1,w-1,h-1);
	
	Translate(x,y);
	VGfloat r = MIN(h,w)*0.45;
	Translate((w)/2,h/2); // Zentrierung des KOS
	
	Stroke(255,140,0,0.5); // Linieninitialisierung
	StrokeWidth(r/50);
	Line(-r,0,r,0);
	
	
	Stroke(0,0,0,0);
	
	// Alt Schreiben
	if (abs(dev) < 20)
		sprintf(text,"%.1f",dev);
	else
		sprintf(text,"%.0f",dev);
	indicator(-r/2, -r/3, length, (int) 0, text);
	
	
	int j = 8;
	sprintf(text,"%d",j);
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
// 	fitwidth(r/7, r/7, 1, text, &tw1);
	Fill(255,255,255,0.2);
	StrokeWidth(r/100);
	for (int i=1; i<=3;i++){
		Stroke(75,75,75,1);
		Fill(0,0,0,0);
		Circle(0,(r/3)*i,r/5);
		
		Circle(0,-(r/3)*i,r/5);
		
		sprintf(text,"%d",(i));
		Fill(75,75,75,1);
		Stroke(0,0,0,0);
		TextMid(0,r/3*i -tw1.fontsize/2, text, tw1.font, tw1.fontsize);
		TextMid(0,-r/3*i-tw1.fontsize/2, text, tw1.font, tw1.fontsize);
	}
	sprintf(text,"10");
	TextEnd(0.98*r,-r, text, tw1.font, tw1.fontsize);
	sprintf(text,"x");
	Text(0.98*r,-r+tw1.fontsize, text, tw1.font, tw1.fontsize/2);
	sprintf(text,"ft");
	Text(r,-r, text, tw1.font, tw1.fontsize);
	
	double plotdev = 0;
	if (dev == 0)
		plotdev = 0;
	if (dev > 0)
		plotdev = log10(dev) * r/3;
	if (dev < 0)
		plotdev = -log10(abs(dev)) * r/3;
	if (dev >= 1000)
		plotdev = log10((double) 999) * r/3;
	if (dev <= -1000)
		plotdev = log10((double) 999) * r/3;
	
	Fill(255,255,255,1);
	Stroke(255,255,255,1); // Linieninitialisierung
	StrokeWidth(r/30);
	Line(-2*r/3,plotdev,2*r/3,plotdev);
	
	Translate(-x,-y);
	Translate(-(w)/2,-h/2); // Zentrierung des KOS
	
	
}

void locplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double x_ist, double x_cmd, double angle){
	char *test = "-1000";
	int length = strlen(test);
	char *text = malloc(length + 1);
	double phimax;
	double r_arc;
	int dir;
	
	
	if (angle > angleold){
		dir = -1; }
	if (angle < angleold){
		dir = 1;}
//		printf("angle: %f, angleold %f, dir %d\n",angle,angleold,dir);
	angleold = angle;
	
	
	Fill(0,0,0,1);
	Rect(x,y,w,h);
	
	Translate(x,y);
	VGfloat r = MIN(h,w)*0.45;
	Translate((w)/2,h/2); // Zentrierung des KOS
	Translate(-r,0);
	
	Stroke(255,140,0,0.5);
	StrokeWidth(r/50);
	
	
	r_arc = 6*r/2;
	if (r_arc <= h*0.95)
		phimax = 90;
	else
		phimax = asin(((double)h*0.95)/r_arc)*57.2958;
	
	Arc(0,0,r_arc,r_arc,-phimax,2*phimax);
	
	Fill(0,0,0,0);
	Stroke(255,255,255,1);
	StrokeWidth(r/30);
	
	r_arc = 3*r*(x_ist/x_cmd);
	if (r_arc <= h*0.8)
		phimax = 90;
	else
		phimax = asin(((double) h*0.8)/r_arc) * 57.2958;
	
	Arc(0,0,r_arc,r_arc,-phimax,2*phimax);
	r_arc /= 2;
	if (dir > 0){
		Line(r_arc,0,r_arc+0.1414*r,-0.1414*r);
		Line(r_arc,0,r_arc-0.1414*r,-0.1414*r);
	}
	if (dir < 0){
		Line(r_arc,0,r_arc+0.1414*r,0.1414*r);
		Line(r_arc,0,r_arc-0.1414*r,0.1414*r);
	}
	
	
	
	Stroke(75,75,75,1);
	StrokeWidth(r/50);
	Line(-r/10,0,r/10,0);
	Line(0,-r/10,0,r/10);
	
	
	
	Stroke(0,0,0,0);
	
// 		FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
	Fill(255,255,255,0.2);
	StrokeWidth(r/100);
	
	// Alt Schreiben
	
	sprintf(text,"%.0f",x_ist);
	indicator(r/2, -r/3, length, (int) 0, text);
	sprintf(text,"%.0f",x_cmd);
	indicator(r/2, r/3, length, (int) 1, text);
	
	
	Translate(r,0);
	Translate(-x,-y);
	Translate(-(w)/2,-h/2); // Zentrierung des KOS
	
	
}

void gammaplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double gamma, double gamma_soll, double gamma_cmd){
	char *test = "-12.2";
	int length = strlen(test);
	char *text = malloc(length + 1);
	
	Fill(0,0,0,1);
	Rect(x+1,y+1,w-1,h-1);
	
	Translate(x,y);
	VGfloat r = h*0.4619;
	Translate((w+r)/2,h/2); // Zentrierung des KOS
//	coordpoint(0,0);
	Fill(0,0,0,0);
	Stroke(75,75,75,1);
	StrokeWidth(r/40);
	Arc(0, 0, 2*r, 2*r, (VGfloat)120, (VGfloat)120);
	
	int j = 88888;
	sprintf(text,"%d",j);
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
// 	fitwidth(w/5, h/5, 1, text, &tw1);
	Fill(255,255,255,1);
	for (int i=0; i<5; i++){
		sprintf(text,"%d",(20-10*i));
		
		Rotate(30);
		Translate(0,r*1.1);
		Rotate(30*(-1-i));
		TextEnd(0,-tw1.fontsize/2, text, tw1.font, tw1.fontsize);
		Rotate(30*(1+i));
		Translate(0,-r*1.1);
		
		Line(0,r,0,r*0.8);
	}
	Rotate(-150);
	
	Stroke(0,0,0,0);
	
	sprintf(text,"%.1f",gamma);
	indicator(0, -r/3, length, (int) 0, text);
	sprintf(text,"%.1f",gamma_cmd);
	indicator(0, r/3, length, (int) 1, text);
	
	double pointgamma_cmd_deg = (double) gamma_cmd * (double)3 - (double)90;
	double pointgamma_soll_deg = (double) gamma_soll * (double)3 - (double)90;
	double pointgamma_deg = (double)gamma * (double)3 - (double)90;
	pointer(0,0,r,pointgamma_deg, pointgamma_cmd_deg, pointgamma_soll_deg);
	
	Translate(-(w+r)/2,-h/2);
	Translate(-x,-y);
}

void phiplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double phi, double phi_cmd, double phi_soll){
	char *test = "-90";
	int length = strlen(test);
	char *text = malloc(length + 1);
	
	Fill(0,0,0,1);
	Rect(x+1,y+1,w-1,h-1);
	
	Translate(x,y);
	Translate(w/2,h/10);
	VGfloat r = 0.4619*w;
	// Zentrierung des KOS
	coordpoint(0,0);
	Fill(0,0,0,0);
	Stroke(75,75,75,1);
	StrokeWidth(r/40);
	Arc(0, 0, 2*r, 2*r, (VGfloat)30, (VGfloat)120);
	
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
	Fill(255,255,255,1);
	
	Rotate(-90);
	for (int i=-2; i<=2; i++){
		sprintf(text,"%d",abs((30*i)));
		
		Rotate(30);
		Translate(0,r*1.1);
		Rotate(30*(-i));
		TextMid(0,0, text, tw1.font, tw1.fontsize);
		Rotate(30*(i));
		Translate(0,-r*1.1);
		
		Line(0,r,0,r*0.8);
	}
	Rotate(-150);
	Rotate(90);
//
	Stroke(0,0,0,0);
	
	sprintf(text,"%2.0f",phi);
	indicator(-r/2, 0.75*h, length, (int) 0, text);
	sprintf(text,"%2.0f",phi_cmd);
	indicator(r/2, 0.75*h, length, (int) 1, text);
//
	pointer(0,0,r,phi, phi_cmd, phi_soll);
	
	Translate(-w/2,-h/10);
	Translate(-x,-y);
}

void vertspd(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double vs, double vs_cmd, double vs_soll){
	char *test = "-2000";
	int length = strlen(test);
	char *text = malloc(length + 1);
// 	int fontsize = 256;
	Fill(0,0,0,1);
	Rect(x+1,y+1,w-1,h-1);
	
	Translate(x,y);
	VGfloat r = h* 0.4619;
	Translate((w+r)/2,h/2); // Zentrierung des KOS
//	coordpoint(0,0);
	Fill(0,0,0,0);
	Stroke(75,75,75,1);
	StrokeWidth(r/40);
	Arc(0, 0, 2*r, 2*r, (VGfloat)120, (VGfloat)120);
	
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
// 	fitwidth(w/5, h/5, 1, text, &tw1);
	Fill(255,255,255,1);
	
	for (int i=0; i<5; i++){
		sprintf(text,"%d",(2000-1000*i));
		
		Rotate(30);
		Translate(0,r*1.1);
		Rotate(30*(-1-i));
		TextEnd(0,-tw1.fontsize/2, text, tw1.font, tw1.fontsize);
		Rotate(30*(1+i));
		Translate(0,-r*1.1);
		
		Line(0,r,0,r*0.8);
	}
	Rotate(-150);
	
	Stroke(0,0,0,0);
	
	// Alt Schreiben
	
	sprintf(text,"%.0f",vs);
	indicator(0, -r/3, length, (int) 0, text);
	sprintf(text,"%.0f",vs_cmd);
	indicator(0, r/3, length, (int) 1, text);
	
	Stroke(0,0,0,0);
	Fill(0,0,0,0);
	double pointvs_cmd_deg = (double) vs_cmd * (double)0.03 - (double)90;
	double pointvs_soll_deg = (double) vs_soll * (double)0.03 - (double)90;
	double pointvs_deg = (double) vs * (double)0.03 - (double)90;
	pointer(0,0,r,pointvs_deg,pointvs_cmd_deg, pointvs_soll_deg);
	
	Translate(-(w+r)/2,-h/2);
	Translate(-x,-y);
}


// 	void circleplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double r_ist, double r_cmd, double bearing) {
// 		char *test = "20000";
//                 int length = strlen(test);
//                 char *text = malloc(length + 1);
//
// 		VGfloat d = MIN(w,h);
// 		VGfloat r = d/2;
// 		VGfloat r_ref = r*3/2;
//
// 		Stroke(0,0,0,0);
// 		StrokeWidth(r/50);
// 		Fill(0,0,0,0.02);
//
// 		Translate(x,y);
// 		Translate(w/2,h/2); // Zentrierung des KOS
//                 Rect(-r,-r,d,d);
//                 Fill(0,0,0,0);
//
//
// 		Stroke(75,75,75,1);
// 		StrokeWidth(r/100);
// 		Circle(0,0,r_ref*0.75);
//                 Circle(0,0,r_ref*1.25);
// 		Line(-r/20,0,r/20,0);
// 		Line(0,-r/20,0,r/20);
//
// 		StrokeWidth(r/50);
// 		Stroke(255,140,0,1);
// 		Circle(0,0,3*r/2);
//
// 		Stroke(0,0,0,0);
// 		Fill(255,255,255,1);
// 		Circle((3*r/4)*(r_ist/r_cmd)*sin(bearing),(3*r/4)*(r_ist/r_cmd)*cos(bearing),r/30);
//
// 		sprintf(text,"%.0f",r_ist);
//                 indicator(0, -r/3, length, (int) 0, text);
//                 sprintf(text,"%.0f",r_cmd);
//                 indicator(0, r/3, length, (int) 1, text);
//
//
// 		Translate(-w/2,-h/2); // Zentrierung des KOS
// 		Translate(-x,-y);
//
// 	}

void altimeter(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double alt, double alt_cmd, double alt_soll) {
	VGfloat d = MIN(w,h) * 0.8;
	VGfloat r = d/2;
	char *test = "12345";
	int length = strlen(test);
	char *text = malloc(length + 1);
	
	Fill(0,0,0,1);
	Rect(x,y,w,h);
	Fill(0,0,0,0);
	
	
	Translate(x,y);
	Translate(w/2,h/2); // Zentrierung des KOS
	
	Stroke(0,0,0,0);
	StrokeWidth(0);
	Fill(75,75,75,1);
	Circle(0,0,2*r*1.01);
	Fill(0,0,0,1);
	Circle(0,0,2*r*0.99);
	
	sprintf(text,"%.0f",alt);
	indicator(0, -r/3, length, (int) 0, text);
	sprintf(text,"%.0f",alt_cmd);
	indicator(0, r/3, length, (int) 1, text);
	
	Stroke(255,255,255,1); // Linieninitialisierung
	StrokeWidth(r/40);
	Fill(255,255,255,1);
	
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
// 	fitwidth(r/3, r/6, 1, text, &tw1);
	
	Stroke(75,75,75,1);
	int max = MAX(alt,MAX(alt_cmd, alt_soll));
	
	if (max <= 9000) {
		if (max <= 4000) {
			for (int i=0; i<=40; i++) {
				Rotate(-i*8);
				Line(0,r,0,r*0.9);
				
				if ((i%5) == 0) {
					sprintf(text,"%d",(i));
					TextMid(0,r*1.05, text, tw1.font, tw1.fontsize);
					Line(0,r,0,r*0.8);
				}
				Rotate(i*8);
			}
			Fill(0,0,0,0);
			double pointalt_cmd_deg = (alt_cmd) *0.08;
			double pointalt_soll_deg = (alt_soll) *0.08;
			double pointalt_deg = (alt) * 0.08;
			pointer(0,0,r,pointalt_deg,pointalt_cmd_deg,pointalt_soll_deg);
		}
		else {
			for (int i=0; i<90; i++) {
				Rotate(-i*4);
				Line(0,r,0,r*0.9);
				
				if ((i%10)==0) {
					sprintf(text,"%d",(i));
					TextMid(0,r*1.05, text, tw1.font, tw1.fontsize);
					Line(0,r,0,r*0.8);
				}
				Rotate(i*4);
			}
			Fill(0,0,0,0);
			double pointalt_cmd_deg = (alt_cmd)*0.04;
			double pointalt_soll_deg = (alt_soll)*0.04;
			double pointalt_deg = (alt)*0.04;
			pointer(0,0,r,pointalt_deg,pointalt_cmd_deg,pointalt_soll_deg);
		}
	}
	
	else {
		for (int i=0; i<18; i++) {
			Rotate(-i*20);
			Line(0,r,0,r*0.8);
			
			if (!(i&1)) {
				sprintf(text,"%d",(i*10));
				TextMid(0,r*1.05, text, tw1.font, tw1.fontsize);
			}
			Rotate(i*20);
		}
		Fill(0,0,0,0);
		double pointalt_cmd_deg = (alt_cmd)*0.02;
		double pointalt_soll_deg = (alt_soll)*0.02;
		double pointalt_deg = (alt) * 0.02;
		pointer(0,0,r,pointalt_deg,pointalt_cmd_deg,pointalt_soll_deg);
	}
	
	Fill(255,255,255,1);
	sprintf(text,"x100ft");
	TextMid(0,-3*r/4, text, tw1.font, tw1.fontsize);
	Translate(-w/2,-h/2); // Zentrierung des KOS
	Translate(-x,-y);
}

void loadplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double load, double load_cmd, double load_soll) {
	fontsize *= 0.5;
	VGfloat d = MIN(w,h) * 0.8;
	VGfloat r = d/2;
	char *test = "100%";
	int length = strlen(test);
	char *text = malloc(length + 1);
	
	
	
	Translate(x,y);
	Translate(w/2,h/10); // Zentrierung des KOS
	
	sprintf(text,"%.0f%%",load);
	indicator(0, r/3, length, (int) 0, text);
// 		sprintf(text,"%.0f",kias_cmd);
// 		indicator(0, r/3, length, (int) 1, text);
	
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
// 	fitwidth(r/3, r/6, 1, text, &tw1);
	Fill(0,0,0,0);
	
	Stroke(255,215,0,1);
	StrokeWidth(r/10);
	//gelber Bogen
	Arc(0,0,r*1.9,r*1.9,(VGfloat) 15,(VGfloat) 12);
	
	StrokeWidth(r/20);
	Stroke(75,75,75,1);
	Arc(0,0,2*r,2*r,(VGfloat) 15,(VGfloat) 150);
	Fill(255,255,255,1);
	Rotate(75);
	for (int i=0; i<=10; i++) {
		Rotate(-i*15);
		if (!(i&1)) {
			sprintf(text,"%d",(i*10));
			TextMid(0,r*1.05, text, tw1.font, tw1.fontsize);
			Line(0,r,0,r*0.8);
		}
		else {
			Line(0,r,0,r*0.9);
		}
		Rotate(i*15);
	}
	Rotate(-75);
	Stroke(180,0,0,1);
	StrokeWidth(r/20);
	Rotate(-75);
	//rote Linie
	Line(0,r*0.9875,0,r*0.8);
	Rotate(75);
	Stroke(0,0,0,0);
	StrokeWidth(r/40);
	
	Fill(0,0,0,0);
	double pointload = 1.5*load-75;
	double pointload_soll = 1.5*load_soll-75;
// 		double pointkias_deg = kias*1.8;
	pointer(0,0,r,pointload,pointload,pointload_soll);
	
	Translate(-w/2,-h/10); // Zentrierung des KOS
	Translate(-x,-y);
	fontsize /= 0.5;
}


void airspd(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double kias, double kias_cmd) {
	VGfloat d = MIN(w,h) * 0.8;
	VGfloat r = d/2;
	char *test = "123";
	int length = strlen(test);
	char *text = malloc(length + 1);
	
	Fill(0,0,0,1);
	Rect(x,y,w,h);
	Fill(0,0,0,0);
	
	
	Translate(x,y);
	Translate(w/2,h/2); // Zentrierung des KOS
	
	sprintf(text,"%.0f",kias);
	indicator(0, -r/3, length, (int) 0, text);
	sprintf(text,"%.0f",kias_cmd);
	indicator(0, r/3, length, (int) 1, text);
	
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
// 	fitwidth(r/3, r/6, 1, text, &tw1);
	Fill(0,0,0,0);
	Stroke(34,139,34,1);
	StrokeWidth(r/10);
	//grÃ¼ner Bogen
	Arc(0,0,r*1.9,r*1.9,(VGfloat) -39.6,(VGfloat) -142.2);
	
	Stroke(255,215,0,1);
	StrokeWidth(r/10);
	//gelber Bogen
	Arc(0,0,r*1.9,r*1.9,(VGfloat) -181.8,(VGfloat) -66.6);
	
	Stroke(255,255,255,1);
	StrokeWidth(r*0.05);
	//weisser Bogen
	Arc(0,0,r*1.8,r*1.8,(VGfloat) -25.2,(VGfloat) -88.2);
	
	Stroke(75,75,75,1);
	StrokeWidth(r/40);
//                Fill(75,75,75,1);
	Fill(0,0,0,0);
	Circle(0,0,2*r);
//                Fill(0,0,0,1);
//                Circle(0,0,2*r*0.99);
	
	Fill(255,255,255,1);
	Stroke(255,255,255,1);
	StrokeWidth(r/40);
	Stroke(75,75,75,1);
	for (int i=0; i<20; i++) {
		Rotate(-i*18);
		if (!(i&1)) {
			sprintf(text,"%d",(i*10));
			TextMid(0,r*1.05, text, tw1.font, tw1.fontsize);
			Line(0,r,0,r*0.8);
		}
		else {
			Line(0,r,0,r*0.9);
		}
		Rotate(i*18);
	}
	Stroke(180,0,0,1);
	StrokeWidth(r/20);
	Rotate(-338.4);
	//rote Linie
	Line(0,r*0.9875,0,r*0.8);
	Rotate(338.4);
	Stroke(0,0,0,0);
	StrokeWidth(r/40);
	
	Fill(255,255,255,1);
	sprintf(text,"KIAS");
	TextMid(0,-0.75*r, text, tw1.font, tw1.fontsize);
	Fill(0,0,0,0);
	double pointkias_cmd_deg = kias_cmd*1.8;
	double pointkias_deg = kias*1.8;
	pointer(0,0,r,pointkias_deg,pointkias_cmd_deg,0);
	
	Translate(-w/2,-h/2); // Zentrierung des KOS
	Translate(-x,-y);
}

void hdgplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double hdg, double hdg_cmd, double hdg_soll) {
	VGfloat d = MIN(w,h) * 0.8;
	VGfloat r = d/2;
	char *test = "345";
	int length = strlen(test);
	char *text = malloc(length + 1);
// 	int fontsize = 256; // StartschriftgrÃƒÂ¶ÃƒÅ¸e setzen
	
	Fill(0,0,0,1);
	Rect(x,y,w,h);
	Fill(0,0,0,0);
	
	Translate(x,y);
	Translate(w/2,h/2); // Zentrierung des KOS
	
	Stroke(0,0,0,0);
	StrokeWidth(0);
	Fill(75,75,75,1);
	Circle(0,0,r*2.02);
	Fill(0,0,0,1);
	Circle(0,0,r*1.98);
	
	sprintf(text,"%.0f",hdg);
	indicator(0, -r/3, length, (int) 0, text);
	sprintf(text,"%.0f",hdg_cmd);
	indicator(0, r/3, length, (int) 1, text);
	
	Stroke(255,255,255,1); // Linieninitialisierung
	StrokeWidth(r/40);
	Fill(255,255,255,1);
	
	FW tw1 = { SansTypeface, 0, fontsize }; //Schrift initialisieren
	
	Stroke(75,75,75,1);
//	Rotate(-90);
	for (int i=0; i<36; i++) {
		Rotate(-i*10);
		if ((i%3)==0) {
			sprintf(text,"%d",(i*10));
			if (i == 0)
				sprintf(text,"N");
			if (i == 9)
				sprintf(text,"O");
			if (i == 18)
				sprintf(text,"S");
			if (i == 27)
				sprintf(text,"W");
			TextMid(0,r*1.05, text, tw1.font, tw1.fontsize);
			Line(0,r,0,r*0.8);
		}
		else
			Line(0,r,0,r*0.9);
		Rotate(i*10);
	}
	Fill(0,0,0,0);
	pointer(0,0,r,hdg,hdg_cmd,hdg_soll);
//	Rotate(90);
	
	Translate(-w/2,-h/2); // Zentrierung des KOS
	Translate(-x,-y);
}

void defaultplot(VGfloat x, VGfloat y, VGfloat w, VGfloat h, double val, double val_cmd) {
	VGfloat r = MIN(w,h);
	char *test = "123456";
	int length = strlen(test);
	char *text = malloc(length + 1);
// 	int fontsize = 256; // StartschriftgrÃƒÂ¶ÃƒÅ¸e setzen
	
	Fill(0,0,0,1);
	Rect(0,0,w,h);
	Fill(0,0,0,0);
	
	Translate(x,y);
	Translate(w/2,h/2);
	Stroke(255,0,0,1);
	StrokeWidth(r/10);
	
	Rotate(45);
	Line(0,-r/2,0,r/2);
	Rotate(-90);
	Line(0,-r/2,0,r/2);
	Rotate(45);
	Stroke(0,0,0,0);
	
	sprintf(text,"%.1f",val);
	indicator(0,-r/3, (int) 3, (int) 0, text);
	sprintf(text,"%.1f",val_cmd);
	indicator(0,r/3, (int) 3, (int) 1, text);
	
	Translate(-w/2,-h/2);
	
	
	
	Translate(-x,-y);
}

void tbl(VGfloat x, VGfloat y, VGfloat w, VGfloat h) {
//	char *test = "Automatic Mode:";
	int length = 255;//strlen(test);
	char *text = malloc(length + 1);
	FW tw1 = { SansTypeface, 0, fontsize };
	
	Translate(x,y);
// 	
// 	switch ((int) state.vert_ctl) {
// 		case 1:
			Translate(0,0.45*h);
			Rotate(-90);
			Fill(0,0,0,1);
			Rect(0,0,w/2,0.45*h);
// 			gammaplot(0,0,w/2,0.425*h,state.vert_ist,state.vert_soll);
			gammaplot(0,0,w/2,0.425*h,state.vert_ist,state.vert_cmd,state.vert_soll);
			Fill(255,255,255,1);
			sprintf(text,"Pitch by ELEV");
			Text(w/20,0.425*h -(tw1.fontsize)/2,text,tw1.font, tw1.fontsize);
			Rotate(90);
			Translate(0,-0.45*h);
// 			break;
// 		case 2:
// 			Translate(0,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			vertspd(0,0,w/2,0.425*h,state.vert_ist,state.vert_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"V/S by ELEV");
// 			Text(w/20,0.425*h -(tw1.fontsize)/2,text,tw1.font, tw1.fontsize);
// 			Translate(0,-0.45*h);
// 			break;
// 		case 3:
// 			Translate(0,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			altimeter(0,0,w/2,0.425*h,state.vert_ist,state.vert_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"ALT by ELEV");
// 			Text(w/20,0.425*h -(tw1.fontsize)/2,text,tw1.font, tw1.fontsize);
// 			Translate(0,-0.45*h);
// 			break;
// 		case 4:
// 			Translate(0,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			gammaplot(0,0,w/2,0.425*h,state.vert_ist,state.vert_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"GS by ELEV");
// 			Text(w/20,0.425*h -(tw1.fontsize)/2,text,tw1.font, tw1.fontsize);
// 			Translate(0,-0.45*h);
// 			break;
// 		default:
// 			Translate(0,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			defaultplot(0,0,w/2,0.425*h,state.vert_ist,state.vert_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"VERT INOP");
// 			Text(w/20,0.425*h -(tw1.fontsize)/2,text,tw1.font, tw1.fontsize);
// 			Translate(0,-0.45*h);
// 			break;
// 	}
// 	
// 	switch ((int) state.lat_ctl) {
// 		
			Translate(w/2,0.45*h);
			Fill(0,0,0,1);
			Rect(0,0,w/2,0.45*h);
//			Rotate(-90);
			gammaplot(0,0,w/2,h*0.425,state.lat_ist,state.lat_soll,state.lat_soll);
//			Rotate(90);
			Fill(255,255,255,1);
			sprintf(text,"BANK by AIL");
			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
			Translate(-w/2,-0.45*h);
// 			break;
			
// 			case 1:
// 			Translate(w/2,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,0.45*h);
// 			hdgplot(0,0,w/2,h*0.425,state.lat_ist,state.lat_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"HDG by RUD");
// 			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			Translate(-w/2,-0.45*h);
// 			break;
// 		case 2:
// 			Translate(w/2,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			phiplot(0,0,w/2,h*0.425,state.lat_ist,state.lat_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"SSLIP->0 by RUD");
// 			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			Translate(-w/2,-0.45*h);
// 			break;
// 		case 3:
// 			Translate(w/2,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			hdgplot(0,0,w/2,h*0.425,state.lat_ist,state.lat_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"TRK by RUD");
// 			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			Translate(-w/2,-0.45*h);
// 			break;
// 		case 4:
// 			Translate(w/2,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			hdgplot(0,0,w/2,h*0.425,state.lat_ist,state.lat_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"HDG by BANK");
// 			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			Translate(-w/2,-0.45*h);
// 			break;
// 		case 5:
// 			Translate(w/2,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			hdgplot(0,0,w/2,h*0.425,state.lat_ist,state.lat_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"TRK by BANK");
// 			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			Translate(-w/2,-0.45*h);
// 			break;
// 		case 999:
// 			Translate(w/2,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			//	circleplot(0,0,w/2,r*0.425,state.lat_ist,state.lat_soll,state.reserve1 * 3.14/180);
// 			locplot(0,0, w/2,h*0.425,state.lat_ist,state.lat_soll,state.reserve1);
// 			Fill(255,255,255,1);
// 			sprintf(text,"CIRC by BANK");
// 			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			Translate(-w/2,-0.45*h);
// 			break;
// 		default:
// 			Translate(w/2,0.45*h);
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			defaultplot(0,0,w/2,h*0.425,state.lat_ist,state.lat_soll);
// 			Fill(255,255,255,1);
// 			sprintf(text,"LAT INOP");
// 			Text(w/20,h*0.425 -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			Translate(-w/2,-0.45*h);
// 			break;
// 	}
// 	
// 	switch ((int) state.spd_ctl) {
// 		case 1:
			Fill(0,0,0,1);
			Rect(0,0,w/2,h*0.45);
			airspd(0,0,w/2,h*0.425,state.spd_ist,state.spd_soll);
			loadplot(0,0,h/5,h/5,state.eng1_pow,state.eng1_pow,state.eng1_pow_soll);
// 			loadplot(w/2 - h/5,0,h/5,h/5,state.eng2_pow,state.eng2_pow);
			Fill(255,255,255,1);
			sprintf(text,"SPD by POW");
			Text(w/20,h*0.425 + -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			break;
// 		case 2:
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			airspd(0,0,w/2,h*0.425,state.spd_ist,state.spd_soll);
// 			loadplot(0,0,h/5,h/5,state.eng1_pow,state.eng1_pow);
// 			loadplot(w/2 - h/5,0,h/5,h/5,state.eng2_pow,state.eng2_pow);
// 			Fill(255,255,255,1);
// 			sprintf(text,"SPD by ELEV");
// 			Text(w/20,h*0.425 + -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			break;
// 		default:
// 			Fill(0,0,0,1);
// 			Rect(0,0,w/2,h*0.45);
// 			defaultplot(0,0,w/2,h*0.425,state.spd_ist,state.spd_soll);
// 			loadplot(0,0,h/5,h/5,state.eng1_pow,state.eng1_pow);
// 			loadplot(w/2 - h/5,0,h/5,h/5,state.eng2_pow,state.eng2_pow);
// 			Fill(255,255,255,1);
// 			sprintf(text,"SPD INOP");
// 			Text(w/20,h*0.425 + -tw1.fontsize/2,text,tw1.font, tw1.fontsize);
// 			break;
// 	}
// 	
// 	Fill(0,0,0,1);
// 	Rect(w/2,0,w/2,h*0.45);
// //		sprintf(text,"GEAR UP");
// //		indicator(3*w/4,((9*h/10)/2)*5/6, (int) 8, (int) 0, text);
// 	if (state.gear_ist == 0)
// 		sprintf(text,"GEAR UP");
// 	else
// 		sprintf(text,"GEAR DN");
// 	indicator(0.75*w,0.3*h, (int) 9, (int) 1, text);
// //		sprintf(text,"FLAPS UP");
// //		indicator(3*w/4,((9*h/10)/2)*2/6, (int) 8, (int) 0, text);
// 	if (state.flaps_ist == 0)
// 		sprintf(text,"FLAPS UP");
// 	if (state.flaps_ist == 1)
// 		sprintf(text,"FLAPS T/O");
// 	if (state.flaps_ist == 2)
// 		sprintf(text,"FLAPS LDG");
// 	
// 	indicator(0.75*w,0.075*h, (int) 9, (int) 1, text);
// 	
// 	Fill(0,0,0,1);
// 	Rect(0,0.9*h,w,0.1*h);
// 	
// 	
// // 		Stroke(255,140,0,1);
// 	Stroke(75,75,75,1);
// 	StrokeWidth(MIN(w,h)/250);
// 	Line(0,0,w,0); // RAHMEN
// 	Line(w,0,w,h);
// 	Line(w,h,0,h);
// 	Line(0,h,0,0);
// 	Line(0,0.9*h/2,w,0.45*h);
// 	Line(w/2,0,w/2,0.9*h);
// 	
// 	
// 	Line(0,0.9*h,w,0.9*h);
// 	Fill(255,255,255,1);
// 	if (state.mode == 1) {
/*
// 		sprintf(text,"Automatic Mode: Segment %.0f - Dist/Time to Next: %.1f [NM] / %.0f%.0f:%.0f%.0f [mm:ss]"\
// // 				,state.segment,state.dist,\
// // 				floor(state.time/60)/10,\
// // 				(double)((int)floor(state.time/60)%(int)10),\
// // 				(double)floor((double)((int)state.time%(int)60)/(double)10),\
// // 				(double)(((int)state.time%(int)60)%(int)10));
// 			,state.segment,state.dist,\
// 			floor(state.time/600),\
// 			(double)((int)floor(state.time/60)%(int)10),\
// 			(double)floor((double)((int)state.time%(int)60)/(double)10),\
// 			(double)(((int)state.time%(int)60)%(int)10));
// 		
// 		if (state.action == 0) {
// 			Fill(0,0,0,0);
// 			StrokeWidth(h/10 * 0.05);
//
 			Translate(9*w/10,h*0.95);*/
// 			Stroke(255,255,255,1);
// 			Line(0,-4*h/100,0,0);
// 			Rotate(30);
// 			Line(0,0,0,4*h/100);
// 			Translate(0,4*h/100);
// 			Line(-1.4142*h/100,-1.4142*h/100,0,-h/10 * 0.025);
// 			Line(1.4142*h/100,-1.4142*h/100,0,-h/10 * 0.025);
// 			Translate(0,-4*h/100);
// 			Rotate(-30);
// 			Stroke(255,140,0,1);
// 			Line(-1.4142*(h/200),-1.4142*(h/200), 1.4142*(h/200), 1.4142*(h/200));
// 			Line(-1.4142*(h/200), 1.4142*(h/200), 1.4142*(h/200),-1.4142*(h/200));
// 			Translate(-9*w/10,-h*0.95);
// 		}
// 		if (state.action == -1) {
// 			Fill(0,0,0,0);
// 			StrokeWidth(h/10 * 0.05);
// 			Translate(9*w/10,h*0.95);
// 			Stroke(255,255,255,1);
// 			Line(0,-4*h/100,0,-2*h/100+(h/10 *0.025));
// 			Arc(0,0,4*h/100,4*h/100,-90,-300);
// 			
// 			Rotate(-120);
// 			Translate(0,2*h/100);
// 			Rotate(-80);
// 			Line(-1.4142*(h/200),-1.4142*(h/200), 1.4142*h/2000,1.4142*h/2000);
// 			Line( 1.4142*(h/200),-1.4142*(h/200),-1.4142*h/2000,1.4142*h/2000);
// 			Rotate(80);
// 			Translate(0,-2*h/100);
// 			Rotate(120);
// 			
// 			Stroke(255,140,0,1);
// 			Line(-1.4142*(h/200),-1.4142*(h/200), 1.4142*(h/200), 1.4142*(h/200));
// 			Line(-1.4142*(h/200), 1.4142*(h/200), 1.4142*(h/200),-1.4142*(h/200));
// 			Translate(-9*w/10,-h*0.95);
// 		}
// 		if (state.action == 1) {
// 			Fill(0,0,0,0);
// 			StrokeWidth(h/10 * 0.05);
// 			Translate(9*w/10,h*0.95);
// 			Stroke(255,255,255,1);
// 			Line(0,-4*h/100,0,-2*h/100+(h/10 *0.025));
// 			Arc(0,0,4*h/100,4*h/100,-90,300);
// 			
// 			Rotate(120);
// 			Translate(0,2*h/100);
// 			Rotate(80);
// 			Line(-1.4142*(h/200),-1.4142*(h/200), 1.4142*h/2000,1.4142*h/2000);
// 			Line( 1.4142*(h/200),-1.4142*(h/200),-1.4142*h/2000,1.4142*h/2000);
// 			Rotate(-80);
// 			Translate(0,-2*h/100);
// 			Rotate(-120);
// 			
// 			Stroke(255,140,0,1);
// 			Line(-1.4142*(h/200),-1.4142*(h/200), 1.4142*(h/200), 1.4142*(h/200));
// 			Line(-1.4142*(h/200), 1.4142*(h/200), 1.4142*(h/200),-1.4142*(h/200));
// 			Translate(-9*w/10,-h*0.95);
// 		}
// 		
// 	}
// 	if (state.mode == 0) {
// 		sprintf(text,"Manual Mode, no SIG avail");
// 	}
// 	if (state.mode == 2) {
// 		sprintf(text,"Initialisation complete - Waiting for signals");
// 	}
// 	
// 	
// 	
// 	Fill(255,255,255,1);
// 	Stroke(0,0,0,1);
// 	TextMid(w/2,0.95*h  - fontsize/2,text,tw1.font, tw1.fontsize);
// 	
	Stroke(0,0,0,0);
	Fill(0,0,0,0);
// 	
// 	
	Translate(-x,-y);
// 	
}

void draw(VGfloat x0, VGfloat y0,VGfloat ws, VGfloat hs) {
	
//     Translate(x0,y0);
	
// 	tbl(x0,y0,ws,hs);
	
	
	
	
// 	int alt = 12345;
// 	int alt_cmd = 10000;
// 	int vs = 654;
// 	int vs_cmd = -1340;
// 	double gamma=-3.378;
// 	double gamma_cmd = 4.1234;
// 	double phi=-13.378;
// 	double phi_cmd = 5.7234;
// 	double hdg = 123;
// 	double hdg_cmd = 270;
// 	double gs_dev =333;
// 	double kias = 123;
// 	double kias_cmd = 120;
// 	double loc_ist;
	
//StrokeWidth(hs/100);
//Stroke(255,255,255,1);
//Rect(0,0,ws,hs);
//Stroke(0,0,0,0);
	
//	for (int alt = 0; alt<=360; alt++){
// 		loc_ist=alt;
// 		gamma=alt;
// 		gs_dev=alt;
//		state.vert_ist = alt;
//		state.lat_ist = alt;
//		state.spd_ist = alt;
// 		altimeter(x0,y0,ws,hs,joy_pos.eta,joy_pos.phi);
//		End();
	//		waituntil(0x1b);
// 		airspd(x0,y0,ws,hs,state.spt_ist,state.spd_soll);
//                 End();
//                waituntil(0x1b);
	
// 		vertspd(x0+ws/2,y0,ws/2,hs/2,vs,vs_cmd);
// //		End();
// //		waituntil(0x1b);
// 		locplot(x0,y0,ws,hs,loc_ist,(VGfloat) 1000);
//		End();
// //              waituntil(0x1b);
// 		gsplot(x0+ws/2,y0+hs/2,ws/2,hs/2,(int) 0,gs_dev);
// 		End();
//		printf("r=%d",alt);
//  		circleplot(x0,y0,ws,hs,(double) alt,2000,(double)0.5* 3.14*alt/180);
//  		End();
// 		waituntil(0x1b);
// 		hdgplot(x0,y0,ws,hs,hdg,hdg_cmd);
//  		End();
// 		usleep((unsigned int) 25000);
//		getrusage(RUSAGE_SELF, &usage);
//		start = usage.ru_utime;
	
//		tbl(x0,y0,ws,hs);
//		End();
//		getrusage(RUSAGE_SELF, &usage);
//		end = usage.ru_utime;
//		printf("Started at: %ld.%lds\n", start.tv_sec, start.tv_usec);
//		printf("Ended at: %ld.%lds\n", end.tv_sec, end.tv_usec);
	
	
//	}
	tbl(x0,y0,ws,hs);
	End();
	
//    return 0;
}

// 	int fiki_init(void) {
// 		int fd;
// 		fd=open("/tmp/fifo_can.1", O_RDONLY);
// 		if(fd == -1)
// 		{
// 			perror("open : ");
// 			exit(0);
// 		}
// 		return fd;
// 	}

// void listen_udp(void) {
// 	// UDPcode START
// 	int sock;
// 	int go = 1;
// 	struct sockaddr_in sa_server;
// 	struct sockaddr_in sa_client;
//
// 	stateinit();
// 	BUF = sizeof(state);
//
// 	unsigned int echolen, clientlen, serverlen;
// 	int received = 0;
//
// 	/* Create the UDP socket */
// 	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
// 		fprintf(stderr, "Failed to create socket");
// 	}
//
// 	/* no blocking */
// 	int flags = fcntl(sock, F_GETFL);
// 	flags |= O_NONBLOCK;
// 	fcntl(sock, F_SETFL, flags);
//
//
// 	/* Construct the server sockaddr_in structure */
// 	memset(&sa_server, 0, sizeof(sa_server));       /* Clear struct */
// 	sa_server.sin_family = AF_INET;                  /* Internet/IP */
// 	sa_server.sin_addr.s_addr = htons(INADDR_ANY);  /* any IP address */
// 	sa_server.sin_port = htons(atoi("5150"));       /* server port */
//
// 	/* Bind the socket */
// 	serverlen = sizeof(sa_server);
// 	if (bind(sock, (struct sockaddr *) &sa_server, serverlen) < 0) {
// 		fprintf(stderr, "Failed to bind server socket");
// 	}
//
// 	/* Connect to remote host */
//
// // 	if (connect(sock,(struct sockaddr *) &sa_server,serverlen) < 0) {
// //		fprintf(stderr, "Failed to connect to server");
// // 	}
//
// 	// UDPcode END
//
// 	// Plotcode START
//
// 	int w, h;
// 	int scale = 90;
// //		printf("vor plot.fiki init\n");
// //	int fd = fiki_init();
// //		printf("nach plot.fiki init\n");
// 	init(&w, &h);
// 	rawterm();
// 	Start(w, h);
// //	int BUF = sizeof(state);
// //	printf("vor plot.while\n");
//
// 	VGfloat ws=w*scale/100;
// 	VGfloat hs=h*scale/100;
// 	VGfloat x0=(100-scale)*w/200;
// 	VGfloat y0=(100-scale)*h/200;
// 	Background(0, 0, 0);
//
// 	fontsize = MIN(ws,hs)/40;
//
// 	draw(x0,y0,ws,hs);
// 	// Plotcode END
// 	int plot = 1;
// 	while(go == 1) {
//
// 		clientlen = sizeof(sa_client);
// 		if ((received = recvfrom(sock, &state, BUF, 0,//)) < 0) {//,
// 			(struct sockaddr *) &sa_client,
// 			&clientlen)) < 0) {
// 			if (plot == 1){
// 				draw(x0,y0,ws,hs);
// 				plot = 0;
// 			}
// 		}
// 		else {
// 			plot = 1;
// 		}
// 	}
// }


void init_joystick(){
	fd_joystick = open ("/dev/input/js0", O_RDONLY | O_NONBLOCK);
}

void listen_joystick(int *plot){
	struct js_event e;
	while (read (fd_joystick, &e, sizeof(e)) > 0) {
		e.type &= ~JS_EVENT_INIT;
		if (e.type == JS_EVENT_AXIS){
			switch (e.number){
				case JS_X:
					joy_pos.eta =  e.value;
					*plot = 1;
					break;
				case JS_Y:
					joy_pos.phi = e.value;
					*plot = 1;
					break;
				case JS_Z:
					joy_pos.xi = e.value;
					break;
				case JS_T:
					joy_pos.thr = e.value;
					*plot = 1;
					break;
				default:
					//nix
					break;
			}
			*plot = 1;
		}
		else if (e.type == JS_EVENT_BUTTON){
			switch (e.number){
				case JS_B0:
					if (e.value == 1){
						send_cmd = 1;
					}
					break;
				case JS_B1:
					if (e.value == 1){
						go = 0;
					}
					break;
				default:
					//nix
					break;
			}
			*plot = 1;
		}
		else{
			//NIX
		}
		/* EAGAIN is returned when the queue is empty */
		if (errno != EAGAIN) {
			printf("Joystick READ ERROR\n");
		}
		//do something interesting with processed events
		
		
	}
}
	
	int main() {
		// PLOTCODE
		int plot = 0;
// 		int *plot_p = &plot;
		int w, h;
		int scale = 90;
		init(&w, &h);
		rawterm();
		Start(w, h);
		VGfloat ws=w*scale/100;
		VGfloat hs=h*scale/100;
		VGfloat x0=(100-scale)*w/200;
		VGfloat y0=(100-scale)*h/200;
		Background(0, 0, 0);
		fontsize = MIN(ws,hs)/40;
		saveterm();
		init_joystick();
		stateinit();
		draw(x0,y0,ws,hs);
		//++PLOT
		while (go == 1){
			listen_joystick(&plot);
// 		if ((received = recvfrom(sock, &state, BUF, 0,//)) < 0) {//,
// 			(struct sockaddr *) &sa_client,
// 			&clientlen)) < 0) {
			if (plot == 1){
				state.vert_soll = map(joy_pos.eta,-32767,32767,20,-20);
				state.lat_soll = map(joy_pos.phi,-32767,32767,-20,20);
				state.spd_soll = map(joy_pos.thr,-32767,32767,85,35);
				draw(x0,y0,ws,hs);
				// Plotcode END
				plot = 0;
			}
		}
// 		else {
// 			plot = 1;
// 		}
		
// 	waituntil(0x1b);
		restoreterm();
		finish();
		return 0;
	}
