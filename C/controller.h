/*
This is the header file to be imported as #include <controller.h>
module definition and initialization
*/
#ifndef CONTROLLER_H
#define CONTROLLER_H

    int update(struct gains *pid, int error, int dt);

#endif