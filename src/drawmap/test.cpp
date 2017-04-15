#include <QApplication>
#include "drawmap.h"
#include <string>
#include <iostream>

using namespace std;

string input("map.log");
string output("map.png");

string rmap;
string fpath;
extern string fuseMap(string , string);
int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	if(argc ==  3){
		for(int i=0;i<argc;i++)
			cout<<argv[i]<<endl;
		input = string(argv[1]);
		output = string(argv[2]);
	}
	else if(argc == 4){
		rmap = string(argv[1]);
		fpath = string(argv[2]);
		input = fuseMap(rmap, fpath);
	}
	CDrawMap dmap;
	dmap.drawLog(input,output);
	return app.exec();
}


