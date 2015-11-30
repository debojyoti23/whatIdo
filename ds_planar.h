#define N1 5000
#define N2 4000
#define min(x,y) (x<y?x:y)
#define max(x,y) (x>y?x:y)
#define N 5
#define N_TRI 0
#define N_CIR 5
typedef pair<double,vector<int> > pairtype;
typedef priority_queue<pairtype,vector<pairtype>,greater<pairtype> > pqtype;
vector<pair<double,double> > edges1[N1]; //edges1[N1*DIM1*2]; //edges1[N1]; for stick version	
vector<pair<double,double> > trackpt1[N1];
vector<pair<double,double> > edges2[N2]; //edges2[N2*DIM2*2]; //edges2[N2]; for stick version
vector<pair<double,double> > trackpt2[N2];
unordered_map<string,bool> doCollide_static[N][N];	//Pairwise static collision status 1=collision,0=none
unordered_map<string,bool> doCollide_dynamic[N][N];	//Pairwise dynamic collision status 1=collision,0=none
class Graph
{
	int V;
public:
	set<pair<int,double> > *adjlst;
	Graph(int v):V(v){adjlst=new set<pair<int,double> >[V];}
	void addEdge(int v1,int v2,double w)
	{
		adjlst[v1].insert(make_pair(v2,w));
		adjlst[v2].insert(make_pair(v1,w));
	}
};
