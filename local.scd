thisProcess.openUDPPort(5555); // the port will stay open until you restart sclang or recompile the class library
// create the OSCFunc
o = OSCFunc({ arg msg, time, addr, recvPort; [msg, time, addr, recvPort].postln; }, '/encoder');
// o.free; // remove the OSCFunc when you are done.