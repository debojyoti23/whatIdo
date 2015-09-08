#include<bits/stdc++.h>
#include<string>
#include<cmath>
#include<unordered_map>
#include<unordered_set>
#include<ctime>
#define DIM1 4
#define DIM2 4
#define N1 5000
#define N2 4000
#define min(x,y) (x<y?x:y)
#define max(x,y) (x>y?x:y)
#define N_TRI 0
#define N_CIR 5
using namespace std;
typedef pair<double,vector<int> > pairtype;
typedef priority_queue<pairtype,vector<pairtype>,greater<pairtype> > pqtype;
vector<pair<double,double> > edges1[N1]; //edges1[N1*DIM1*2]; //edges1[N1]; for stick version	
vector<pair<double,double> > trackpt1[N1];
vector<pair<double,double> > edges2[N2]; //edges2[N2*DIM2*2]; //edges2[N2]; for stick version
vector<pair<double,double> > trackpt2[N2];
int IS_THICK=0;
class Graph
{
	int V;
public:
	vector<pair<int,double> > *adjlst;
	Graph(int v):V(v){adjlst=new vector<pair<int,double> >[V];}
	void addEdge(int v1,int v2,double w)
	{
		adjlst[v1].push_back(make_pair(v2,w) );
		//adjlst[v2].push_back(make_pair(v1,w) );
	}
};
class compare{
public:
	bool operator()(pair<string,int> e1,pair<string,int> e2){
		return e1.second>e2.second;
	}
};
// bool comparefn(pairtype e1,pairtype e2){
// 	return e1.first<=e2.first;
// }
void customsort(vector<pairtype> &v)
{
	vector<pair<double,int> > aux;
	for(int i=0;i<v.size();i++){
		aux.push_back(make_pair(v[i].first,i));
	}
	sort(aux.begin(),aux.end());
	vector<pairtype> temp;
	for(int i=0;i<v.size();i++){
		temp.push_back(v[aux[i].second]);
	}
	v=temp;
}
inline string formKey(vector<int> robots)
{
	string temp=to_string(robots[0]);
	for(int i=1;i<robots.size();i++){
		temp=temp+"-"+to_string(robots[i]);
	}
	return temp;
}
bool onSegment(pair<double,double> p,pair<double,double> q,pair<double,double> r)
{
    if (q.first <= max(p.first, r.first) && q.first >= min(p.first, r.first) &&
        q.second <= max(p.second, r.second) && q.second >= min(p.second, r.second))
       return true;
 
    return false;
}
int orientation(pair<double,double> p,pair<double,double> q,pair<double,double> r)
{
	int val = (q.second - p.second) * (r.first - q.first) -
              (q.first - p.first) * (r.second - q.second); 
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2;  // clock or counterclock wise
}
bool doIntersect(pair<double,double> segments1[][2],pair<double,double> segments2[][2],int n1,int n2)
{
	for(int i=0;i<n1;i++)
	for(int j=0;j<n2;j++)
	{
		int o1=orientation(segments1[i][0],segments1[i][1],segments2[j][0]);
		int o2=orientation(segments1[i][0],segments1[i][1],segments2[j][1]);
		int o3=orientation(segments2[j][0],segments2[j][1],segments1[i][0]);
		int o4=orientation(segments2[j][0],segments2[j][1],segments1[i][1]);
		if(o1!=o2 && o3!=o4)
			return true;
		//special case, on segment scenario
		if(o1==0 && onSegment(segments1[i][0],segments2[j][0],segments1[i][1])) return true;
		if(o2==0 && onSegment(segments1[i][0],segments2[j][1],segments1[i][1])) return true;
		if(o3==0 && onSegment(segments2[j][0],segments1[i][0],segments2[j][1])) return true;
		if(o4==0 && onSegment(segments2[j][0],segments1[i][1],segments2[j][1])) return true;
	}
	return false;
}
bool isFeasible_static(pair<int,int> tworobots,int type[])
{
	vector<pair<double,double> > e[2];
	for(int i=0;i<2;i++){
		if(type[i]==0){
			e[i]=(i==0)?edges1[tworobots.first]:edges1[tworobots.second];
		}
		else{
			e[i]=(i==0)?edges2[tworobots.first]:edges2[tworobots.second];
		}
	}
	// first point repeats at the last in e1,e2
	int n1=e[0].size()-1,n2=e[1].size()-1;
	pair<double,double> r1_edges[n1][2];	//each pair is of the form (startpoint,endpoint) for a line-segment
	pair<double,double> r2_edges[n2][2];
	for(int i=0;i<n1;i++){
		r1_edges[i][0]=e[0][i];
		r1_edges[i][1]=e[0][i+1];
	}
	for(int i=0;i<n2;i++){
		r2_edges[i][0]=e[1][i];
		r2_edges[i][1]=e[1][i+1];
	}
	return !doIntersect(r1_edges,r2_edges,n1,n2);
}
bool isFeasible_static_arm(pair<int,int> tworobots,int type[])
{
	// Check for overlap of two robots by checking edge intersection
	int n_edges1=2*DIM1;
	int n_edges2=2*DIM2;
	int r1=tworobots.first,r2=tworobots.second;
	pair<double,double> r1_edges[n_edges1][2];	//each pair is of the form (startpoint,endpoint) for a line-segment
	pair<double,double> r2_edges[n_edges2][2];
	for(int i=0;i<n_edges1;i++)
	{
		r1_edges[i][0]=edges1[r1*n_edges1+i][0];
		r1_edges[i][1]=edges1[r1*n_edges1+i][1];
	}
	for(int i=0;i<n_edges2;i++)
	{
		r2_edges[i][0]=edges2[r2*n_edges2+i][0];
		r2_edges[i][1]=edges2[r2*n_edges2+i][1];	
	}
	return !doIntersect(r1_edges,r2_edges,n_edges1,n_edges2);
}
bool isFeasible_static_multi(vector<int> robots,bool mask[],int type[])
{
	for(int i=0;i<robots.size()-1;i++){
		for(int j=i+1;j<robots.size();j++){
			if(mask[i] || mask[j])continue;
			int category[2];
			category[0]=type[i];category[1]=type[j];
			if(!isFeasible_static(make_pair(robots[i],robots[j]),category))
				return false;
		}
	}
	return true;
}
bool isFeasible_dynamic(pair<int,int> tworobots_from,pair<int,int> tworobots_to,int type[])
{
	// robotType: Triangle=1, Circle=2
	// Check for collision between two robots while in transition
	vector<pair<double,double> > e_from[2],e_to[2];
	vector<pair<double,double> > tp_from[2],tp_to[2];
	for(int i=0;i<2;i++){
		if(type[i]==0){
			e_from[i]=(i==0)?edges1[tworobots_from.first]:edges1[tworobots_from.second];
			e_to[i]=(i==0)?edges1[tworobots_from.first]:edges1[tworobots_to.second];
			tp_from[i]=(i==0)?trackpt1[tworobots_from.first]:trackpt1[tworobots_from.second];
			tp_to[i]=(i==0)?trackpt1[tworobots_from.first]:trackpt1[tworobots_to.second];
		}
		else{
			e_from[i]=(i==0)?edges2[tworobots_from.first]:edges2[tworobots_from.second];
			e_to[i]=(i==0)?edges2[tworobots_from.first]:edges2[tworobots_to.second];
			tp_from[i]=(i==0)?trackpt2[tworobots_from.first]:trackpt2[tworobots_from.second];
			tp_to[i]=(i==0)?trackpt2[tworobots_from.first]:trackpt2[tworobots_to.second];
		}
	}
	int n1_edge=e_from[0].size()-1,n2_edge=e_from[1].size()-1;
	int n1_track=tp_from[0].size(),n2_track=tp_from[1].size();
	//Edges
	pair<double,double> r1_from_edges[n1_edge][2],r2_from_edges[n2_edge][2];
	pair<double,double> r1_to_edges[n1_edge][2],r2_to_edges[n2_edge][2];
	for(int i=0;i<n1_edge;i++){
		r1_from_edges[i][0]=e_from[0][i];
		r1_from_edges[i][1]=e_from[0][i+1];
		r1_to_edges[i][0]=e_to[0][i];
		r1_to_edges[i][1]=e_to[0][i+1];
	}
	for(int i=0;i<n2_edge;i++){
		r2_from_edges[i][0]=e_from[1][i];
		r2_from_edges[i][1]=e_from[1][i+1];
		r2_to_edges[i][0]=e_to[1][i];
		r2_to_edges[i][1]=e_to[1][i+1];
	}
	// 2)Tracks
	pair<double,double> r1_tracks[n1_track][2];
	pair<double,double> r2_tracks[n2_track][2];
	if(tworobots_from.first!=tworobots_to.first){
		for(int i=0;i<n1_track;i++){
			r1_tracks[i][0]=tp_from[0][i];
			r1_tracks[i][1]=tp_to[0][i];
		}
	}
	if(tworobots_from.second!=tworobots_to.second){
		for(int i=0;i<n2_track;i++){
			r2_tracks[i][0]=tp_from[1][i];
			r2_tracks[i][1]=tp_to[1][i];
		}
	}
	// Detecting Track-track collision
	if(tworobots_from.first!=tworobots_to.first && tworobots_from.second!=tworobots_to.second){
		if(doIntersect(r1_tracks,r2_tracks,n1_track,n2_track))return false;
	}
	// Detecting Track-Edge collison
	if(tworobots_from.first!=tworobots_to.first){
		if(doIntersect(r1_tracks,r2_from_edges,n1_track,n2_edge))return false;
		if(tworobots_from.second!=tworobots_to.second){
			if(doIntersect(r1_tracks,r2_to_edges,n1_track,n2_edge))return false;
		}
	}
	if(tworobots_from.second!=tworobots_to.second){
		if(doIntersect(r1_from_edges,r2_tracks,n1_edge,n2_track))return false;
		if(tworobots_from.first!=tworobots_to.first){
			if(doIntersect(r1_to_edges,r2_tracks,n1_edge,n2_track))return false;
		}
	}
	return true;
}
bool isFeasible_dynamic_arm(pair<int,int> tworobots_from,pair<int,int> tworobots_to,int type[])
{
	// Check for collision between two robots while in transition
	int r1_from=tworobots_from.first,r2_from=tworobots_from.second;
	int r1_to=tworobots_to.first,r2_to=tworobots_to.second;
	// 1)Edges
	int n_edges1=2*DIM1;
	int n_edges2=2*DIM2;
	pair<double,double> r1_from_edges[n_edges1][2],r2_from_edges[n_edges2][2],r1_to_edges[n_edges1][2],r2_to_edges[n_edges2][2];
	for(int i=0;i<n_edges1;i++)
	{
		r1_from_edges[i][0]=edges1[r1_from*n_edges1+i][0];
		r1_from_edges[i][1]=edges1[r1_from*n_edges1+i][1];
		r1_to_edges[i][0]=edges1[r1_to*n_edges1+i][0];
		r1_to_edges[i][1]=edges1[r1_to*n_edges1+i][1];
	}
	for(int i=0;i<n_edges2;i++)
	{
		r2_from_edges[i][0]=edges2[r2_from*n_edges2+i][0];
		r2_from_edges[i][1]=edges2[r2_from*n_edges2+i][1];
		r2_to_edges[i][0]=edges2[r2_to*n_edges2+i][0];
		r2_to_edges[i][1]=edges2[r2_to*n_edges2+i][1];
	}
	// 2)Tracks
	int n_trackpt1=trackpt1[0].size();
	int n_trackpt2=trackpt2[0].size();
	pair<double,double> r1_tracks[n_trackpt1][2];
	pair<double,double> r2_tracks[n_trackpt2][2];
	if(tworobots_from.first!=tworobots_to.first){
		for(int i=0;i<n_trackpt1;i++){
			r1_tracks[i][0]=trackpt1[r1_from][i];
			r1_tracks[i][1]=trackpt1[r1_to][i];
		}
	}
	if(tworobots_from.second!=tworobots_to.second){
		for(int i=0;i<n_trackpt2;i++){
			r2_tracks[i][0]=trackpt2[r2_from][i];
			r2_tracks[i][1]=trackpt2[r2_to][i];
		}
	}
	// Detecting Track-track collision
	if(tworobots_from.first!=tworobots_to.first && tworobots_from.second!=tworobots_to.second){
		if(doIntersect(r1_tracks,r2_tracks,n_trackpt1,n_trackpt2))return false;
	}
	// Detecting Track-Edge collison
	if(tworobots_from.first!=tworobots_to.first){
		if(doIntersect(r1_tracks,r2_from_edges,n_trackpt1,n_edges2))return false;
		if(tworobots_from.second!=tworobots_to.second)
		if(doIntersect(r1_tracks,r2_to_edges,n_trackpt1,n_edges2))return false;
	}
	if(tworobots_from.second!=tworobots_to.second){
		if(doIntersect(r1_from_edges,r2_tracks,n_edges1,n_trackpt2))return false;
		if(tworobots_from.first!=tworobots_to.first)
		if(doIntersect(r1_to_edges,r2_tracks,n_edges1,n_trackpt2))return false;
	}
	return true;
}
bool isFeasible_dynamic_multi(vector<int> robots_from,vector<int> robots_to,bool mask[],int type[])
{
	for(int i=0;i<robots_from.size()-1;i++){
		for(int j=i+1;j<robots_from.size();j++){
			if(mask[i] || mask[j])continue;
			int category[2];
			category[0]=type[i];category[1]=type[j];
			if(!isFeasible_dynamic(make_pair(robots_from[i],robots_from[j]),make_pair(robots_to[i],robots_to[j]),category))
			return false;
		}
	}
	return true;
}
vector<int> findIndividualPath(Graph g,int src,int dst)
{
	unordered_set<int> taken;
	unordered_map<int,double> inpq;
	unordered_map<int,int> parents;
	priority_queue<pair<double,int>,vector<pair<double,int> >,greater<pair<double,int> > > pq;
	inpq[src]=0;
	pq.push(make_pair(0,src));
	parents[src]=src;
	cout<<"Individual path:"<<src<<"-"<<dst<<endl;
	while(!pq.empty())
	{
		pair<double,int> temp=pq.top();
		pq.pop();
		int from=temp.second;
		if(from==dst)
			break;
		if(taken.find(from)!=taken.end())
			continue;
		double curdist=temp.first;
		vector<pair<int,double> > nbr;
		nbr=g.adjlst[from];
		for(int i=0;i<nbr.size();i++){
			int to=nbr[i].first;
			double dist=curdist+nbr[i].second;
			if(taken.find(to)!=taken.end())
				continue;
			if(inpq.find(to)==inpq.end() || dist<inpq[to]){
				pq.push(make_pair(dist,to));
				inpq[to]=dist;
				parents[to]=from;
			}
		}
	}
	vector<int> path;
	if(parents.find(dst)==parents.end())
		return path;
	int temp=dst;
	while(parents[temp]!=temp){
		path.insert(path.begin(),temp);
		temp=parents[temp];
	}
	path.insert(path.begin(),src);
	// Print path
	for(int i=0;i<path.size();i++)
		cout<<path[i]<<" ";
	cout<<endl;
	cout<<"Path length: "<<path.size()-1<<endl;
	return path;
}
bool isless(vector<int> v1,vector<int> v2)
{
	// check if v1<=v2
	for(int i=0;i<v1.size();i++){
		if(v1[i]>v2[i])
			return false;
	}
	return true;
}
vector<int> getRechability(Graph g1,Graph g2,vector<int> src,vector<int> dst,vector<int> paths[],unordered_map<string,pair<string,int> > &cost,bool mask[],int type[])
{
	vector<vector<int> > minimal_set;
	// bool valid[paths[0].size()][paths[1].size()][paths[2].size()][paths[3].size()];
	// memset(valid,true,sizeof(valid));
	int l[src.size()];
	vector<int> goal;
	for(int i=0;i<src.size();i++){
		if(!mask[i]){
			goal.push_back(paths[i].back());
			l[i]=paths[i].size();
		}
		else{
			goal.push_back(paths[i][0]);
			l[i]=1;
		}
	}
	cost[formKey(goal)]=make_pair(formKey(goal),-1);

	for(int p1=l[0]-1;p1>=0;p1--)
	for(int p2=l[1]-1;p2>=0;p2--)
	for(int p3=l[2]-1;p3>=0;p3--)
	for(int p4=l[3]-1;p4>=0;p4--)
	for(int p5=l[4]-1;p5>=0;p5--){
		vector<int> vfrom,vfrom_ind;
		vfrom.push_back(paths[0][p1]);vfrom.push_back(paths[1][p2]);vfrom.push_back(paths[2][p3]);vfrom.push_back(paths[3][p4]);vfrom.push_back(paths[4][p5]);
		vfrom_ind.push_back(p1);vfrom_ind.push_back(p2);vfrom_ind.push_back(p3);vfrom_ind.push_back(p4);vfrom_ind.push_back(p5);
		string from=formKey(vfrom);
		//check for static collision
		if(!isFeasible_static_multi(vfrom,mask,type)){	
			// valid[p1][p2][p3][p4]=false;
			continue;
		}
		priority_queue<pair<string,int>,vector<pair<string,int> >,compare> qnbr;
		// Check Neighbours for all robots
		for(int i1=0;i1<=1 && p1+i1<l[0];i1++)
		for(int i2=0;i2<=1 && p2+i2<l[1];i2++)
		for(int i3=0;i3<=1 && p3+i3<l[2];i3++)
		for(int i4=0;i4<=1 && p4+i4<l[3];i4++)
		for(int i5=0;i5<=1 && p5+i5<l[4];i5++)
		{
			vector<int> vto;
			vto.push_back(paths[0][p1+i1]);vto.push_back(paths[1][p2+i2]);vto.push_back(paths[2][p3+i3]);vto.push_back(paths[3][p4+i4]);vto.push_back(paths[4][p5+i5]);
			string to=formKey(vto);
			if(cost.find(to)==cost.end())
				continue;
			if(!isFeasible_dynamic_multi(vfrom,vto,mask,type))
				continue;
			qnbr.push(make_pair(to,cost[to].second));
		}
		if(!qnbr.empty()){
			cost[from]=make_pair(qnbr.top().first,1+qnbr.top().second);
			int flag=0;
			if(!minimal_set.empty()){
				vector<vector<int> >::iterator it;
				vector<vector<int> > temp;
				for(it=minimal_set.begin();it!=minimal_set.end();it++){
					if(isless(vfrom_ind,*it))
						continue;
					if(isless(*it,vfrom_ind)){
						flag=1;
						break;
					}
					temp.push_back(*it);
				}
				if(flag!=1)
					minimal_set=temp;
			}
			if(minimal_set.empty() || flag!=1){
				minimal_set.push_back(vfrom_ind);
			}
		}
	}
	vector<int> nonzero(src.size(),0);
	cout<<"States with no predecessor:\n";
	for(int i=0;i<minimal_set.size();i++){
		cout<<formKey(minimal_set[i])<<endl;
		for(int j=0;j<src.size();j++)
			nonzero[j]+=minimal_set[i][j];
	}
	return nonzero;
}
int findCompositePath(Graph g1,Graph g2,vector<int> src,vector<int> dst,int type[],vector<int> paths[])
{
	unordered_set<string> taken;
	pqtype pq;
	bool mask[src.size()];
	memset(mask,false,sizeof(mask));
	///int total_edges=0,discarded_edges=0;
	unordered_map<string,double> inpq;
	unordered_map<string,vector<int> > parents;
	int k=12; // value of k depends on the policy of choosing neigbours
	pq.push(make_pair(0,src));
	inpq[formKey(src)]=0;
	parents[formKey(src)]=src;
	//int max=0;
	while(!pq.empty())
	{
		// if(max<pq.size())
		// 	max=pq.size();
		pairtype temp=pq.top();
		pq.pop();
		vector<int> from=temp.second;
		// Check if from=dst
		bool seen=true;
		for(int i=0;i<src.size();i++){
			if(from[i]!=dst[i]){
				seen=false;break;
			}
		}
		if(seen)break;
		double curdist=temp.first;
		if(taken.find(formKey(from))!=taken.end())
			continue;
		taken.insert(formKey(from));
		vector<pair<int,double> > nbr[src.size()];
		for(int i=0;i<src.size();i++){
			if(type[i]==0)
				nbr[i]=g1.adjlst[from[i]];
			else
				nbr[i]=g2.adjlst[from[i]];
		}
		vector<pairtype> selected,productMat;
		for(int i=0;i<src.size();i++){
			vector<int> temp=from;
			for(int j=0;j<nbr[i].size();j++){
				temp[i]=nbr[i][j].first;
				selected.push_back(make_pair(nbr[i][j].second,temp));
			}
		}
		if(src.size()==2){
			for(int i=0;i<nbr[0].size();i++)
			for(int j=0;j<nbr[1].size();j++){
				double distance=sqrt(nbr[0][i].second*nbr[0][i].second+nbr[1][j].second*nbr[1][j].second);
				vector<int> temp;
				temp.push_back(nbr[0][i].first);temp.push_back(nbr[1][j].first);
				productMat.push_back(make_pair(distance,temp));
			}	
		}
		else if(src.size()==3){
			for(int i=0;i<nbr[0].size();i++)
			for(int j=0;j<nbr[1].size();j++)
			for(int k=0;k<nbr[2].size();k++){
				double distance=sqrt(nbr[0][i].second*nbr[0][i].second+nbr[1][j].second*nbr[1][j].second+nbr[2][k].second*nbr[2][k].second);
				vector<int> temp;
				temp.push_back(nbr[0][i].first);temp.push_back(nbr[1][j].first);temp.push_back(nbr[2][k].first);
				productMat.push_back(make_pair(distance,temp));
			}	
		}
		else if(src.size()==4){
			for(int i=0;i<nbr[0].size();i++)
			for(int j=0;j<nbr[1].size();j++)
			for(int k=0;k<nbr[2].size();k++)
			for(int l=0;l<nbr[3].size();l++){
				double distance=sqrt(nbr[0][i].second*nbr[0][i].second+nbr[1][j].second*nbr[1][j].second+nbr[2][k].second*nbr[2][k].second+nbr[3][l].second*nbr[3][l].second);
				vector<int> temp;
				temp.push_back(nbr[0][i].first);temp.push_back(nbr[1][j].first);temp.push_back(nbr[2][k].first);temp.push_back(nbr[3][l].first);
				productMat.push_back(make_pair(distance,temp));
			}	
		}
		else{
			cout<<"Unable to handle! Max size of collision group is four...\n";
			return 0;
		}
		customsort(productMat);
		// try{
		// 	sort(productMat.begin(),productMat.end(),comparefn);
		// }catch(exception &e){
		// 	cerr<<"Exception caught "<<e.what()<<endl;
		// 	for(int i=0;i<from.size();i++){
		// 		cout<<from[i]<<" ";
		// 	}
		// 	cout<<endl;
		// 	cout<<productMat.size()<<" throwing...\n";
		// 	throw e;
		// }
		for(int i=0;i<min(k,productMat.size());i++)
		{
			selected.push_back(productMat[i]);							//distance
			//selected.push_back(make_pair(1,productMat[i].second));	//step
		}
		for(int i=0;i<selected.size();i++)
		{
			vector<int> to=selected[i].second;
			double wt=selected[i].first;
			if(taken.find(formKey(to))!=taken.end())
				continue;
			double distance=curdist+wt;
			if((inpq.find(formKey(to))==inpq.end() && isFeasible_static_multi(to,mask,type)) || (inpq.find(formKey(to))!=inpq.end() && inpq[formKey(to)]>distance)){
				//total_edges++;
				if(isFeasible_dynamic_multi(from,to,mask,type)){
					pq.push(make_pair(distance,to));
					inpq[formKey(to)]=distance;
					parents[formKey(to)]=from;
				}
			}
		}
	}
	vector<int> temp=dst;
	stack<vector<int> > stk;
	if(parents.find(formKey(dst))==parents.end()){
		return 0;
	}
	//Storing individual paths in function argument
	while(1)
	{
		stk.push(temp);
		if(temp==parents[formKey(temp)])
		break;
		temp=parents[formKey(temp)];
	}
	while(!stk.empty())
	{
	    vector<int> temp=stk.top();
	    stk.pop();
	    for(int i=0;i<temp.size();i++){
	    	if(paths[i].empty())
	    		paths[i].push_back(temp[i]);
	    	else if(paths[i].back()!=temp[i])
	    		paths[i].push_back(temp[i]);
	    }
	}
	return 1;
}
int findJointShortestPath(Graph g1,Graph g2,vector<int> src,vector<int> dst,int type[])
{
	cout<<"Source: "<<formKey(src)<<endl;
	cout<<"Destination: "<<formKey(dst)<<endl;
	bool mask[src.size()];
	memset(mask,false,sizeof(mask));
	if(!isFeasible_static_multi(src,mask,type))
	{
		cout<<"Conflicting Start configuration!\n";
		return 0;
	}
	else if(!isFeasible_static_multi(dst,mask,type))
	{
		cout<<"Conflicting Goal configuration!\n";
		return 0;
	}
	vector<int> paths[src.size()];
	for(int i=0;i<src.size();i++){
		if(type[i]==0)
			paths[i]=findIndividualPath(g1,src[i],dst[i]);
		else
			paths[i]=findIndividualPath(g2,src[i],dst[i]);
		// Check for existence of individual paths
		if(paths[i].size()==0){
			cout<<"Disconnected Component!\n";
			return 0;
		}
	}	
	bool collision[src.size()];
	bool cause_deadlock[src.size()];
	memset(cause_deadlock,false,sizeof(cause_deadlock));
	memset(collision,false,sizeof(collision));
	vector<int> minConnectedState;
	string start=formKey(src);
	string goal=formKey(dst);
	vector<vector<int> > collision_buckets;
	int flag=0,bucket_count=0;
	int collision_bucket_num[src.size()];
	memset(collision_bucket_num,-1,sizeof(collision_bucket_num));

	while(1){
		unordered_map<string,pair<string,int> > cost;	//<from,<to,steps-to-take>>
		for(int i=0;i<src.size();i++)
		if(collision[i]){
			mask[i]=true;
			cause_deadlock[i]=true;
			break;
		}
		minConnectedState=getRechability(g1,g2,src,dst,paths,cost,mask,type);
		if(cost.find(start)!=cost.end())
		{	
			if(flag==0){	//First attempt is deadlock free, all the robots can follow their individual paths
				string temp=start;
				cout<<"Joint Path:"<<endl<<start<<endl;
				while(temp!=goal){
					cout<<cost[temp].first<<endl;
					temp=cost[temp].first;
				}
				cout<<"SUCCESS! Fixed path coordination cost="<<cost[start].second<<endl;
				return 2;
			}
			else{	// Remaining robots are free from deadlock
				vector<int> temp;	
				for(int i=0;i<src.size();i++){
					if(collision[i]){
						collision_bucket_num[i]=bucket_count;
						temp.push_back(i);
					}
				}				
				collision_buckets.push_back(temp);
				bucket_count++;
				break;
			}
		}
		else{	//Deadlock situation
			if(flag==0){
				cout<<"Fixed path coordination FAILURE! Resolving collision...\n";
				for(int i=0;i<src.size();i++)
					if(minConnectedState[i]!=0)
						collision[i]=true;
				flag=1;
			}
			else{
				// Remove one of the colliding robots and check for deadlock
				vector<int> temp;
				for(int i=0;i<src.size();i++)
					if(collision[i] && minConnectedState[i]==0){
						collision[i]=false;
						temp.push_back(i);
					}
				if(temp.size()>1){	//Deadlock does not dissolve
					collision_buckets.push_back(temp);
					for(int i=0;i<temp.size();i++)
						collision_bucket_num[temp[i]]=bucket_count;
					bucket_count++;
				}
			}
		}
	}

	//Get the collision groups
	cout<<"1. Number of Collision groups: "<<collision_buckets.size()<<endl;
	for(int i=0;i<collision_buckets.size();i++){
		for(int j=0;j<collision_buckets[i].size();j++){
			cout<<collision_buckets[i][j]<<" ";
		}
		cout<<endl;
	}

	// Assigning to collision buckets...adding one by one to the collision free set of robots
	for(int i=0;i<src.size();i++){
		if(cause_deadlock[i]==false)
			continue;
		mask[i]=false;

		// // Mask status
		// cout<<"Mask status:\n";
		// for(int i=0;i<src.size();i++)
		// 	cout<<mask[i]<<" ";
		// cout<<endl;
		unordered_map<string,pair<string,int> > cost;	//<from,<to,steps-to-take>>
		minConnectedState=getRechability(g1,g2,src,dst,paths,cost,mask,type);
		int flag=0,bucketno;
		vector<int> temp;
		for(int j=0;j<src.size();j++){
			if(minConnectedState[j]!=0){
				flag=1;
				mask[j]=true;
				cause_deadlock[j]=false;
				temp.push_back(j);
				if(collision_bucket_num[j]!=-1)
					bucketno=collision_bucket_num[j];
			}
		}
		if(flag==1){
			collision_buckets[bucketno]=temp;
		}
	}
	//Get the collision groups
	cout<<"2. Number of Collision groups: "<<collision_buckets.size()<<endl;
	for(int i=0;i<collision_buckets.size();i++){
		for(int j=0;j<collision_buckets[i].size();j++){
			cout<<collision_buckets[i][j]<<" ";
		}
		cout<<endl;
	}
	//Composite planning for colliding robots
	for(int i=0;i<collision_buckets.size();i++){
		vector<int> start,end,index;
		index=collision_buckets[i];		
		int category[index.size()];
		vector<int> paths_new[index.size()];
		for(int j=0;j<index.size();j++){
			start.push_back(src[index[j]]);
			end.push_back(dst[index[j]]);
			category[j]=type[index[j]];
		}
		int res=findCompositePath(g1,g2,start,end,category,paths_new);
		if(res==0){
			cout<<"No path exist\n";
			return 1;
		}
		for(int j=0;j<index.size();j++)
			paths[index[j]]=paths_new[j];
	}
	return 2;
}
void readKNN(ifstream &fnbr,ifstream &fd,ifstream &fvalid,Graph g)
{
	string snbr,sd,svalid;
	cout<<"Building Roadmap Graph..."<<endl;
	while(1)
	{
		string temp,temp1,nbr;
		if(!getline(fnbr,snbr))
			break;
		getline(fd,sd);
		getline(fvalid,svalid);
	   	stringstream stnbr(snbr);
	   	stringstream std(sd);
	   	stringstream stvalid(svalid);
	   	getline(stvalid,temp,',');	   	
	   	if(stoi(temp)==0)	//use -std=c++0x
	   		continue;
	   	getline(stnbr,temp,',');
	   	int src=stoi(temp);	   	
	   	getline(std,temp,',');  	
	   	while(getline(stnbr,nbr,','))
	   	{
	   		getline(stvalid,temp,',');
	   		getline(std,temp1,',');
	   		if(stoi(temp)==0)
	   			continue;
	   		//g.addEdge(src-1,stoi(nbr)-1,1);
	   		g.addEdge(src-1,stoi(nbr)-1,stof(temp1));
	   	}
	}
	//cout<<"Individual roadmap graph created!\n";
}
vector<int> create_tuple(int num,...)
{
	vector<int> vec;
	va_list valist;
	va_start(valist,num);
	for(int i=0;i<num;i++)
		vec.push_back(va_arg(valist,int));
	va_end(valist);
	return vec;
}
int main(int argc,char *argv[])
{
	Graph g1(N1);
	Graph g2(N2);
	string name="arena3";
	string fstr="/home/debojyoti/Dropbox/Navigation/"+name+"/csv/";
	string fstr1="/home/debojyoti/Dropbox/Navigation/"+name+"/triangle/";
	string fstr2="/home/debojyoti/Dropbox/Navigation/"+name+"/circle/";
	string temp;
	clock_t t,lastTime;
	
	// Load neighbourhood, distance and validity Robot 1
	ifstream fnbr1((fstr+name+"_nbr1.csv").c_str());
	ifstream fd1((fstr+name+"_d1.csv").c_str());
	ifstream fvalid1((fstr+name+"_validity1.csv").c_str());
	if(fnbr1.is_open() && fd1.is_open() && fvalid1.is_open())
	readKNN(fnbr1,fd1,fvalid1,g1);
	else
	{cerr<<"Problem opening KNN file!\n";return 0;}
	// Load Edges and Trackpoints of Robot1
	ifstream fedges1(fstr1+"edges.csv");
	ifstream ftrackpt1(fstr1+"trackpoints.csv");
	string line;
	int i=0;
	if(!fedges1.is_open() || !ftrackpt1.is_open())
	{
		cerr<<"Problem opening Edge and Trackpoint files!\n";
		return 0;
	}
	while(getline(fedges1,line))
	{
		stringstream ss(line);
		string temp;
		while(getline(ss,temp,','))
		{
			double x=stof(temp);
			getline(ss,temp,',');
			double y=stof(temp);
			edges1[i].push_back(make_pair(x,y));
		}
		i++;
	}
	i=0;
	while(getline(ftrackpt1,line))
	{
		stringstream ss(line);
		string temp;
		while(getline(ss,temp,','))
		{
			double x=stof(temp);
			getline(ss,temp,',');
			double y=stof(temp);
			trackpt1[i].push_back(make_pair(x,y));
		}
		i++;
	}
	// Load neighbourhood, distance and validity Robot 2
	ifstream fnbr2((fstr+name+"_nbr2.csv").c_str());
	ifstream fd2((fstr+name+"_d2.csv").c_str());
	ifstream fvalid2((fstr+name+"_validity2.csv").c_str());
	if(fnbr2.is_open() && fd2.is_open() && fvalid2.is_open())
	readKNN(fnbr2,fd2,fvalid2,g2);
	else
	{cerr<<"Problem opening KNN file!\n";return 0;}
	// Load Edges and Trackpoints of Robot2
	ifstream fedges2(fstr2+"edges.csv");
	ifstream ftrackpt2(fstr2+"trackpoints.csv");
	if(!fedges2.is_open() || !ftrackpt2.is_open())
	{
		cerr<<"Problem opening Edge and Trackpoint files!\n";
		return 0;
	}
	i=0;
	while(getline(fedges2,line))
	{
		stringstream ss(line);
		string temp;
		while(getline(ss,temp,','))
		{
			double x=stof(temp);
			getline(ss,temp,',');
			double y=stof(temp);
			edges2[i].push_back(make_pair(x,y));
		}
		i++;
	}
	i=0;
	while(getline(ftrackpt2,line))
	{
		stringstream ss(line);
		string temp;
		while(getline(ss,temp,','))
		{
			double x=stof(temp);
			getline(ss,temp,',');
			double y=stof(temp);
			trackpt2[i].push_back(make_pair(x,y));
		}
		i++;
	}
	// Take source and destination as input, each as a pair of two robot instances
	vector<int> src,dst; //index starts from 0
	/*src=make_pair(1992,174);//make_pair(37,391);
	dst=make_pair(5,188);//make_pair(181,37);	
	if(argc==5)
	{
		src=make_pair(atoi(argv[1]),atoi(argv[2])),dst=make_pair(atoi(argv[3]),atoi(argv[4]));		//index starts from 0
	}*/
	/* Randomly choose valid src and destination */
	// Read collision files
	ifstream fcollision1(fstr1+"collision.csv");
	ifstream fcollision2(fstr2+"collision.csv");
	vector<int> goodIndices1,goodIndices2;
	i=0;
	while(getline(fcollision1,line))
	{
		if(stoi(line)==0)
			goodIndices1.push_back(i);
		i++;
	}
	i=0;
	while(getline(fcollision2,line))
	{
		if(stoi(line)==0)
			goodIndices2.push_back(i);
		i++;
	}
	i=0;
	int totalRun=1;
	srand(time(NULL));
	int failure_count=0;
	double heapSize_avg=0;
	int heapSize_max=0;
	double discardedEdgeRatio=0;
	float average_clockTicks;
	float max_clockTicks=0;
	t=clock();
	lastTime=t;
	int type[]={1,1,1,1,1};
	while(i<totalRun){
		cout<<"Iteration "<<i+1<<endl;
		// Tuple consists of one triangle followed by three circle instances
		//src=create_tuple(4,goodIndices1[rand()%goodIndices1.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()]);
		//dst=create_tuple(4,goodIndices1[rand()%goodIndices1.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()]);
		src=create_tuple(5,82,1170,13,61,1290);
		//dst=create_tuple(5,60,1130,1186,96,1246);	//3+2 robots deadlock
		//dst=create_tuple(5,60,1130,73,96,1294);	//collision free
		dst=create_tuple(5,60,1130,1186,160,1246);	//2+2 robot deadlock
		//dst=create_tuple(5,103,160,1186,96);	//2 robot deadlock

		pair<double,double> spaceComplexity;	//<heapsize,ratio_of_discardedEdge>
		int ret=findJointShortestPath(g1,g2,src,dst,type);
		if(ret==0)
			continue;
		else if(ret==1)
			failure_count++;
		// heapSize_avg+=spaceComplexity.first/totalRun;
		// if(spaceComplexity.first>heapSize_max)
		// 	heapSize_max=spaceComplexity.first;
		// if((temp1-lastTime)>max_clockTicks)
		// 	max_clockTicks=temp1;
		// discardedEdgeRatio+=spaceComplexity.second;
		clock_t temp1=clock();
		cout<<"Time taken: "<<(float)(temp1-lastTime)/CLOCKS_PER_SEC<<" seconds\n";
		lastTime=temp1;
		i++;
	}
	t=clock()-t;
	average_clockTicks=(float)t/totalRun;
	// discardedEdgeRatio/=totalRun;
	// double successRate=1-failure_count/(double)totalRun;
	// Report Section
	// cout<<"************* Report *************"<<endl;
	// cout<<"Total Run				:"<<totalRun<<endl;
	// cout<<"Average running time		:"<<(float)average_clockTicks/CLOCKS_PER_SEC<<" seconds"<<endl;
	// cout<<"Maximum running time		:"<<(float)max_clockTicks/CLOCKS_PER_SEC<<" seconds"<<endl;
	// cout<<"Success rate: 			:"<<successRate<<endl;
	// cout<<"Average heap size: 		:"<<heapSize_avg<<endl;
	// cout<<"Maximum heap size: 		:"<<heapSize_max<<endl;
	// cout<<"Ratio of Discarded Edges :"<<discardedEdgeRatio<<endl;
	// cout<<"********* End of Report **********"<<endl;
}
