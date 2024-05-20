#include "main.h"

//externs
double targ;
bool manual = false, cataPIDEnable = true;

double power, error, deriv;

void cataPID(void* ignore){
    printf("Cata PID started\n");
	Motor cata(cataPort, cataGearset, cataEncoder);
    Controller master(CONTROLLER_MASTER);

    cata.tare_position();

    while(true){
        error = targ - cata.get_position();
        power = error*mechlibKP;
        
        if (cataPIDEnable == true){
            cata.move(power);
            if (power < 20 || error < 9) cataPIDEnable = false;
        } else if (manual || master.get_digital(DIGITAL_Y)){
            cata.move(110);
            targ = cata.get_position();
            manual = false;
        } else {
            cata.move(0);
        }
        printf("cataPIDEnable: %s\ntarg: %f, pos: %f, error: %f\n", (cataPIDEnable)? "true" : "false", targ, cata.get_position(), error);
        delay(15);
    }
}

// double assignment to stop race condition
void shoot(int num){
    targ += 540*num;
    cataPIDEnable = true;
    delay(500);
    cataPIDEnable = true;
}