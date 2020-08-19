// File:          MTRN4110_PhaseD.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <stdio.h>
#include <windows.h>
#include <MTRN4110_PhaseB.cpp>
#include <MTRN4110_PhaseA.cpp>
#include <iostream>
#define MAP_TEXT_COLS 37
#define MAP_TEXT_ROWS 11
#define MAP_FILE_NAME "../../Map.txt"

//using namespace webots;
using namespace std;

int MTRN4110_PhaseA(char pathPlan[]);
int MTRN4110_PhaseB(char maptext[][MAP_TEXT_COLS], char pathPlan[]);

int main(int argc, char **argv) {
char pathPlan[50];
char maptext[MAP_TEXT_ROWS][MAP_TEXT_COLS];

  cout << "Beginning Phase C" << endl; 
  STARTUPINFO si;
  PROCESS_INFORMATION pi;

  ZeroMemory( &si, sizeof(si) );
  si.cb = sizeof(si);
  ZeroMemory( &pi, sizeof(pi) );
  
  char cmdline [] = "ipython MTRN4110_PhaseC.py";
  
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
  while(MTRN4110_PhaseB(maptext, pathPlan)) {}
  cout << "Phase B Complete" << endl;
  cout << "Beginning Phase A" << endl;
  while(MTRN4110_PhaseA(pathPlan)) {}
  cout << "Phase A Complete" << endl;
  return 0;
}