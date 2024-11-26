; Enable infrared communication on Thymio
; Author: Sven Wolff

    dc end_toc                      ; total size of event handler table
    dc _ev.init, init               ; id and address of init event
end_toc:

init:                               ; code executed on init event
    push.s 1                        ; push address of the arg, stored somewhere in free memory
    store _userdata
    callnat _nf.prox.comm.enable    ; call native function to enable infrared communication
    stop                            ; stop program
