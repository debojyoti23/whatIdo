#define N1 5000
#define N2 5000
#define N 5
#define min(x,y) (x<y?x:y)
#define max(x,y) (x>y?x:y)
using namespace std;
typedef pair<double,vector<int> > pairtype;
typedef priority_queue<pairtype,vector<pairtype>,greater<pairtype> > pqtype;
int dim[] = {4,4,4,4,4};
vector<pair<double,double> > edges[N][N1*4*2];	
vector<pair<double,double> > trackpt[N][N1];
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