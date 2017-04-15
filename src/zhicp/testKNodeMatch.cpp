#include "ZHIcp_Warpper.h"
#include <vector>
#include <fcntl.h>
#include "mrpt/slam.h"
#include "mrpt/poses.h"
#include <fstream>
#include <string.h>
#include <stdlib.h>

using namespace std;

bool readScanXYP_bin(const char* fn, vector<float>& x, vector<float> & y, double p[3])
{
    FILE* fp = fopen(fn, "r"); 
    if(fp == NULL)
    {
        return false;
    }
    int n ;
    if(fread(&n, sizeof(int), 1, fp) != 1)
    {
        return false;
    }
    x.resize(n); 
    y.resize(n); 
    if(fread(&x[0], sizeof(float)*n, 1, fp) != 1)
    {
        return false; 
    }
    if(fread(&y[0], sizeof(float)*n, 1, fp) != 1)
    {
        return false;
    }
    if(fread(p, sizeof(p), 1, fp) !=1)
    {
        return false;
    }
    return true;
}

bool readScanXYP(const char* fn, vector<float>& x, vector<float>& y, double p[3])
{
    ifstream inf(fn); 
    if(!inf.is_open())
    {
        return false;
    }
    string delim(", \t");
    char line[8192*2];
    if(!inf.getline(line, sizeof(line)))
        return false;
    int n = atoi(strtok(line,delim.c_str()));
    assert(n>0 && "n <= 0");
    x.resize(n);
    y.resize(n); 
    if(!inf.getline(line, sizeof(line)))
        return false;
    x[0] = atof(strtok(line, delim.c_str()));
    for(int i=1; i<n; i++)
        x[i] = atof(strtok(NULL, delim.c_str()));
    if(!inf.getline(line, sizeof(line)))
        return false;
    y[0] = atof(strtok(line, delim.c_str()));
    for(int i=1; i<n; i++)
        y[i] = atof(strtok(NULL, delim.c_str()));
    if(!inf.getline(line, sizeof(line)))
        return false;
    p[0] = atof(strtok(line, delim.c_str()));
    p[1] = atof(strtok(NULL, delim.c_str()));
    p[2] = atof(strtok(NULL, delim.c_str()));
    cout<<"read number of points: "<<n<<endl;
    return true;
}

void test1();
using mrpt::poses::CPose2D;

int main(int argc, char* argv[])
{
    test1();
    cout<<"finished test1()"<<endl;
    return (0); 
}

void test1()
{
    vector<float> rx, ry, ax, ay; 
    double p[3];     
    double in[3];
    double out[3];
    CPose2D p1, p2; 
    CICPWarpper icp;
    icp.setDebugWindowOn();
    for(int i=1; i<10; i++)
    {
        char fname[255]; 
        sprintf(fname, "n%d_scan.txt", i);
        if(i==1)
        {
            if(!readScanXYP(fname, rx, ry, p)) 
//            if(!readScanXYP_bin(fname, rx, ry, p)) 
            {
                cout<<"failed to load map scan!"<<endl;
                return ;
            }
            p1 = CPose2D(p[0], p[1], p[2]); 
            continue ;
        }
       if(!readScanXYP(fname, ax, ay, p))
//        if(!readScanXYP_bin(fname, ax, ay, p))
        {
            cout<<"failed to load node scan: "<<fname<<endl; 
            return ;
        }
        p2 = CPose2D(p[0], p[1], p[2]);
        cout<<"input p1: "<<p1<<" p2: "<<p2<<endl;
        // CPose2D rel = p1-p2;
        CPose2D rel = p2-p1;
        cout<<"input rel: "<<rel<<endl;
        in[0] = rel[0]; in[1] = rel[1]; in[2] = rel[2]; 
        float goodness = icp.ICPMatch(&rx[0], &ry[0], rx.size(), &ax[0], &ay[0], ax.size(), in, out);
        cout<<"match node: "<<i<<" with key_node 0: score: "<<goodness<<endl;
        cout<<endl;
    }
}


