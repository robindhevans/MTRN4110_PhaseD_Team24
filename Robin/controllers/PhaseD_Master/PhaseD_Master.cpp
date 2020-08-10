// File:          PhaseD_Master.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <stdio.h>
#include <windows.h>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <z5197018_MTRN4110_PhaseB.cpp>
#include <z5197018_MTRN4110_PhaseA.cpp>
#include <iostream>


//using namespace webots;
using namespace std;

int z5197018_MTRN4110_PhaseA();
int z5197018_MTRN4110_PhaseB();

int main(int argc, char **argv) {

  cout << "Beginning Phase C" << endl; 
  STARTUPINFO si;
  PROCESS_INFORMATION pi;

  ZeroMemory( &si, sizeof(si) );
  si.cb = sizeof(si);
  ZeroMemory( &pi, sizeof(pi) );
  
  char cmdline [] = "ipython z5197018_MTRN4110_PhaseC.py";
  
  LPSTR command = &cmdline[0];
  
  if( !CreateProcess( NULL,   // No module name (use command line)
        command,        // Command line
        NULL,           // Process handle not inheritable
        NULL,           // Thread handle not inheritable
        FALSE,          // Set handle inheritance to FALSE
        0,              // No creation flags
        NULL,           // Use parent's environment block
        NULL,           // Use parent's starting directory 
        &si,            // Pointer to STARTUPINFO structure
        &pi )           // Pointer to PROCESS_INFORMATION structure
    ) 
    {
        cout << "CreateProcess failed" << endl;
        return 1;
    }
    // Wait until child process exits.
    WaitForSingleObject( pi.hProcess, INFINITE );

    // Close process and thread handles. 
    CloseHandle( pi.hProcess );
    CloseHandle( pi.hThread );
  cout << "Phase C complete" << endl;
  cout << "Beginning Phase B" << endl;
  while(z5197018_MTRN4110_PhaseB()) {}
  cout << "Phase B Complete" << endl;
  cout << "Beginning Phase A" << endl;
  while(z5197018_MTRN4110_PhaseA()) {}
  cout << "Phase A Complete" << endl;
  return 0;
}