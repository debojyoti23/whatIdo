#include<bits/stdc++.h>
#include<string>
#include<cmath>
#include<unordered_map>
#include<unordered_set>
#include<ctime>
#include "ds_arm.h"
#define ROBOTICARM
#ifdef ROBOTICARM
	#include "collision_handler_arm.h"
#else
	#include "collision_handler_planar.h"
#endif
using namespace std;
void showtime()
{
	time_t t=time(0);
	struct tm *now=localtime(&t);
	cout << (now->tm_hour) << ':' 
         << (now->tm_min) << ':'
         <<  now->tm_sec
         << endl;
}
class compare{
public:
	bool operator()(pair<string,int> e1,pair<string,int> e2){
		return e1.second>e2.second;
	}
};
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
	for(int i=1;i<robots.size();i++)
		temp=temp+"-"+to_string(robots[i]);
	return temp;
}
inline string formKey_indexed(vector<int> robots,vector<int> positions)
{
	string temp=to_string(robots[0]);
	string temp1=to_string(positions[0]);
	for(int i=1;i<robots.size();i++){
		temp=temp+"-"+to_string(robots[i]);
		temp1=temp1+"-"+to_string(positions[i]);
	}
	temp=temp+":"+temp1;
	return temp;
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
		set<pair<int,double> > nbr;
		nbr=g.adjlst[from];
		set<pair<int,double> >::iterator it;
		for(it=nbr.begin();it!=nbr.end();it++){
			int to=it->first;
			double dist=curdist+it->second;
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
vector<int> expand(Graph &g1,Graph &g2,int size,int K,unordered_set<string> &taken,pqtype &pq,unordered_map<string,double> &inpq,unordered_map<string,vector<int> > &parents,int type[],bool mask[])
{
	// Returns extracted node from the priority queue
	pairtype temp=pq.top();
	pq.pop();
	vector<int> from=temp.second;
	double curdist=temp.first;
	if(taken.find(formKey(from))!=taken.end())
		return from;
	taken.insert(formKey(from));
	set<pair<int,double> > nbr[size];
	for(int i=0;i<size;i++){
		if(type[i]==0){
			nbr[i]=g1.adjlst[from[i]];
			nbr[i].insert(make_pair(from[i],0));
		}
		else{
			nbr[i]=g2.adjlst[from[i]];
			nbr[i].insert(make_pair(from[i],0));
		}
	}
	// Finding neighbours of 'from' node in composite space by filtering out from composite nbrhood matrix
	vector<pairtype> selected,productMat;
	// Select Axial elements first
	for(int i=0;i<size;i++){
		vector<int> temp=from;
		set<pair<int,double> >::iterator j;
		for(j=nbr[i].begin();j!=nbr[i].end();j++){
			temp[i]=j->first;
			selected.push_back(make_pair(j->second,temp));
		}
	}	
	set<pair<int,double> >::iterator i,j,k,l;
	// Next step is to select best suitable elements from the rest of product matrix. We define
	// composite neighbour as follows: $Y \in nbr(X)$ if $Y(i)==nbr(X(i))$ or $X(i)$ itself for
	// all $i$.
	/************** TO BE FIXED! MISSING SOME ELEMENTS ************/
	if(size==2){
		for(i=nbr[0].begin();i!=nbr[0].end();i++)
		for(j=nbr[1].begin();j!=nbr[1].end();j++){
			if(i->first==from[0] && j->first==from[1])
				continue;
			double distance=sqrt(i->second*i->second+j->second*j->second);
			vector<int> temp;
			temp.push_back(i->first);temp.push_back(j->first);
			productMat.push_back(make_pair(distance,temp));
		}
	}
	else if(size==3){
		for(i=nbr[0].begin();i!=nbr[0].end();i++)
		for(j=nbr[1].begin();j!=nbr[1].end();j++)
		for(k=nbr[2].begin();k!=nbr[2].end();k++){
			if(i->first==from[0] && j->first==from[1] && k->first==from[2])
				continue;
			double distance=sqrt(i->second*i->second+j->second*j->second+k->second*k->second);
			vector<int> temp;
			temp.push_back(i->first);temp.push_back(j->first);temp.push_back(k->first);
			productMat.push_back(make_pair(distance,temp));
		}
	}
	else if(size==4){
		for(i=nbr[0].begin();i!=nbr[0].end();i++)
		for(j=nbr[1].begin();j!=nbr[1].end();j++)
		for(k=nbr[2].begin();k!=nbr[2].end();k++)
		for(l=nbr[3].begin();l!=nbr[3].end();l++){
			if(i->first==from[0] && j->first==from[1] && k->first==from[2] && l->first==from[3])
				continue;
			double distance=sqrt(i->second*i->second+j->second*j->second+k->second*k->second+l->second*l->second);
			vector<int> temp;
			temp.push_back(i->first);temp.push_back(j->first);temp.push_back(k->first);temp.push_back(l->first);
			productMat.push_back(make_pair(distance,temp));
		}	
	}
	// Choose k elements from productMat with minimum distance
	customsort(productMat);
	for(int ii=0;ii<min(K,productMat.size());ii++)
	{
		selected.push_back(productMat[ii]);							//distance
		//selected.push_back(make_pair(1,productMat[ii].second));	//step
	}
	for(int ii=0;ii<selected.size();ii++)
	{
		vector<int> to=selected[ii].second;
		double wt=selected[ii].first;
		if(taken.find(formKey(to))!=taken.end())
			continue;
		double distance=curdist+wt;
		if((inpq.find(formKey(to))==inpq.end() && isFeasible_static_multi(to,mask,type)) || (inpq.find(formKey(to))!=inpq.end() && inpq[formKey(to)]>distance)){
			if(isFeasible_dynamic_multi(from,to,mask,type)){
				pq.push(make_pair(distance,to));
				inpq[formKey(to)]=distance;
				parents[formKey(to)]=from;
			}
		}
	}
	return from;
}
int findCompositePath_bidirectional(Graph g1,Graph g2,vector<int> src,vector<int> dst,int type[],vector<int> paths[])
{
	if(src.size()>4){
		cout<<"Can't compute composite path for more than 4 robots\n";
		return 0;
	}
	unordered_set<string> taken_src,taken_dst;
	pqtype pq_src,pq_dst;
	bool mask[src.size()];
	memset(mask,false,sizeof(mask));
	unordered_map<string,double> inpq_src,inpq_dst;
	unordered_map<string,vector<int> > parents_src,parents_dst;
	int k=12; 	// value of k depends on the policy of choosing neigbours
	pq_src.push(make_pair(0,src));pq_dst.push(make_pair(0,dst));
	inpq_src[formKey(src)]=0;inpq_dst[formKey(dst)]=0;
	parents_src[formKey(src)]=src;parents_dst[formKey(dst)]=dst;
	string src_str=formKey(src),dst_str=formKey(dst);
	stack<vector<int> > stk;
	cout<<"Roadmap composition started at ";
	showtime();
	while(!pq_src.empty() && !pq_dst.empty())
	{
		// Randomly choose direction of this iteration
		vector<int> res;
		int t=rand()%2;
		if(t==0){
			res=expand(g1,g2,src.size(),k,taken_src,pq_src,inpq_src,parents_src,type,mask);
			// If src tree is connected to dst node
			if(taken_src.find(dst_str)!=taken_src.end()){
				vector<int> temp=res;
				// Saves path from src to res
				while(1)
				{
					stk.push(temp);
					if(temp==parents_src[formKey(temp)])
						break;
					temp=parents_src[formKey(temp)];
				}
				while(!stk.empty())
				{
				    temp=stk.top();
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
			// Check if connection established via intermediate res node
			if(taken_dst.find(formKey(res))!=taken_dst.end()){
				vector<int> temp=res;
				// Saves path from src to res
				while(1)
				{
					stk.push(temp);
					if(temp==parents_src[formKey(temp)])
						break;
					temp=parents_src[formKey(temp)];
				}
				while(!stk.empty())
				{
				    temp=stk.top();
				    stk.pop();
				    for(int i=0;i<temp.size();i++){
				    	if(paths[i].empty())
				    		paths[i].push_back(temp[i]);
				    	else if(paths[i].back()!=temp[i])
				    		paths[i].push_back(temp[i]);
				    }
				}
				// Saves path from res to dst
				temp=parents_dst[formKey(res)];
				while(1){
					for(int i=0;i<temp.size();i++){
				    	if(paths[i].back()!=temp[i])
				    		paths[i].push_back(temp[i]);
				    }
				    if(parents_dst[formKey(temp)]==temp){
				    	showtime();
				    	return 1;
				    }
				    temp=parents_dst[formKey(temp)];
				}
			}
		}
		else{
			res=expand(g1,g2,src.size(),k,taken_dst,pq_dst,inpq_dst,parents_dst,type,mask);
			// If dst tree is connected to src node
			if(taken_dst.find(src_str)!=taken_dst.end()){
				vector<int> temp=res;
				while(1)
				{
					for(int i=0;i<temp.size();i++){
				    	if(paths[i].empty())
				    		paths[i].push_back(temp[i]);
				    	else if(paths[i].back()!=temp[i])
				    		paths[i].push_back(temp[i]);
				    }
					if(temp==parents_dst[formKey(temp)]){
						showtime();
						return 1;
					}
					temp=parents_dst[formKey(temp)];
				}
			}
			// Check if connection established via intermediate res node
			if(taken_src.find(formKey(res))!=taken_src.end()){
				vector<int> temp=res;
				// Saves path from src to res
				while(1)
				{
					stk.push(temp);
					if(temp==parents_src[formKey(temp)])
						break;
					temp=parents_src[formKey(temp)];
				}
				while(!stk.empty())
				{
				    temp=stk.top();
				    stk.pop();
				    for(int i=0;i<temp.size();i++){
				    	if(paths[i].empty())
				    		paths[i].push_back(temp[i]);
				    	else if(paths[i].back()!=temp[i])
				    		paths[i].push_back(temp[i]);
				    }
				}
				// Saves path from res to dst
				temp=parents_dst[formKey(res)];
				while(1){
					for(int i=0;i<temp.size();i++){
				    	if(paths[i].back()!=temp[i])
				    		paths[i].push_back(temp[i]);
				    }
				    if(parents_dst[formKey(temp)]==temp){
				    	showtime();
				    	return 1;
				    }
				    temp=parents_dst[formKey(temp)];
				}
			}
		}
	}
	showtime();
	return 0;
}
vector<int> reverseVec(vector<int> v)
{
	vector<int> r;
	for(int i=v.size()-1;i>=0;i--){
		r.push_back(v[i]);
	}
	return r;
}
void print_collision_groups(vector<vector<int> > groups)
{
	cout<<"Collision groups:\n";
	for(int i=0;i<groups.size();i++){
		vector<int> temp=groups[i];
		for(int j=0;j<temp.size();j++)
			cout<<temp[j]<<" ";
		cout<<endl;
	}
}
void find_connected_components(int matrix[N][N],vector<vector<int> > &collision_buckets)
{
	//BFS
	bool visited[N];
	memset(visited,false,sizeof(visited));
	for(int i=0;i<N;i++){
		if(visited[i])
			continue;
		queue<int> temp;
		vector<int> group;
		temp.push(i);
		while(!temp.empty()){
			int now=temp.front();
			temp.pop();
			group.push_back(now);
			visited[now]=true;
			for(int j=0;j<N;j++){
				if(!visited[j] && matrix[now][j]==1){
					temp.push(j);
				}
			}
		}
		if(group.size()>1)
			collision_buckets.push_back(group);
	}
}
int findJointShortestPath(Graph g1,Graph g2,vector<int> src,vector<int> dst,int type[])
{
	cout<<src.size()<<endl;
	cout<<"Source: "<<formKey(src)<<endl;
	cout<<"Destination: "<<formKey(dst)<<endl;
	bool mask[src.size()];
	memset(mask,false,sizeof(mask));
	if(!isFeasible_static_multi(src,mask,type)){
		cout<<"Conflicting Start configuration!\n";
		return 0;
	}
	else if(!isFeasible_static_multi(dst,mask,type)){
		cout<<"Conflicting Goal configuration!\n";
		return 0;
	}
	vector<int> paths[src.size()],paths_rev[src.size()];
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
	bool initial_deadlocked[src.size()];
	vector<int> minConnectedState;
	vector<int> src_ind,dst_ind;
	for(int i=0;i<src.size();i++){
		src_ind.push_back(0);
		dst_ind.push_back(paths[i].size()-1);
	}
	string start=formKey_indexed(src,src_ind);
	string goal=formKey_indexed(dst,dst_ind);
	while(1){ // Each time, with modified individual paths
		vector<vector<int> > reachable_states_src,reachable_states_dst;
		int flag=0,bucket_count=0;
		memset(collision,false,sizeof(collision));
		memset(initial_deadlocked,false,sizeof(initial_deadlocked));
		memset(mask,false,sizeof(mask));
		bool isfirst=true;
		vector<vector<int> > temp_rch;
		int deadlock_matrix[N][N];
		memset(deadlock_matrix,0,sizeof(deadlock_matrix));
		vector<int> unresolved;
		int removed;
		while(1){// iterating to find collision groupss
			unordered_map<string,pair<string,int> > cost;	//<from,<to,steps-to-take>>
			for(int i=0;i<src.size();i++)
			if(initial_deadlocked[i]){
				mask[i]=true;
				initial_deadlocked[i]=false;
				collision[i]=false;
				removed=i;
				cout<<"Removed "<<i<<"..."<<endl;
				break;
			}
			cout<<"From End: ";
			minConnectedState=getReachability(g1,g2,src,dst,paths,cost,mask,type,temp_rch);
			// FRontier from end
			if(!flag){
				reachable_states_dst=temp_rch;
				for(int i=0;i<src.size();i++){
					if(minConnectedState[i]==0)
						mask[i]=true;
				}
			}			
			// Frontier from start
			if(!flag && cost.find(start)==cost.end()){
				unordered_map<string,pair<string,int> > cost;
				for(int i=0;i<src.size();i++){
					paths_rev[i]=reverseVec(paths[i]);
				}
				cout<<"From Start[Reversing the paths]:";
				getReachability(g1,g2,dst,src,paths_rev,cost,mask,type,temp_rch);
				cout<<"Corrected states with no successor:\n";
				for(int i=0;i<temp_rch.size();i++){
					for(int j=0;j<src.size();j++){
						temp_rch[i][j]=paths[j].size()-temp_rch[i][j]-1;
					}
					cout<<formKey(temp_rch[i])<<endl;
				}
				reachable_states_src=temp_rch;
			}
			if(cost.find(start)!=cost.end()){
				if(flag==0){	//First attempt is deadlock free, all the robots can follow their individual paths
					int x;
					string temp=start,out;
					cout<<"Joint Path:"<<endl;
					do{
						stringstream ss(temp);
						getline(ss,out,':');
						cout<<out<<endl;
						x=cost[temp].second;
						temp=cost[temp].first;
					}while(x!=0);
					cout<<"SUCCESS! Coordination cost="<<cost[start].second<<endl;
					return 2;
				}
				else{	// Remaining robots are free from deadlock
					vector<int> temp;	
					for(int i=0;i<src.size();i++){
						if(collision[i]){
							temp.push_back(i);
							cout<<"Freed:"<<i<<endl;
							collision[i]=false;
						}
					}
					temp.push_back(removed);
					for(int i=0;i<temp.size();i++)
						for(int j=i+1;j<temp.size();j++){
							deadlock_matrix[temp[i]][temp[j]]=1;
							deadlock_matrix[temp[j]][temp[i]]=1;
						}
					break;
				}
			}
			else{	//Deadlock situation
				if(flag==0){
					cout<<"FAILURE! Robots in deadlock: ";
					for(int i=0;i<src.size();i++)
						if(minConnectedState[i]!=0){
							initial_deadlocked[i]=true;
							collision[i]=true;
							cout<<i<<" ";
						}
					cout<<endl;
					flag=1;
				}
				else{
					// Remove one of the colliding robots and check for deadlock
					vector<int> temp;
					for(int i=0;i<src.size();i++)
						if(collision[i] && minConnectedState[i]==0){
							temp.push_back(i);
							cout<<"Freed:"<<i<<endl;
							collision[i]=false;
						}
					if(temp.size()>0){	//Deadlock does not dissolve
						temp.push_back(removed);
						for(int i=0;i<temp.size();i++)
							for(int j=i+1;j<temp.size();j++){
								deadlock_matrix[temp[i]][temp[j]]=1;
								deadlock_matrix[temp[j]][temp[i]]=1;
							}
					}
					else{
						cout<<"Unable to free any\n";
						unresolved.push_back(removed);
					}
				}
			}
		}
		// Get collision groups by finding connected components in deadlock_matrix
		cout<<"Deadlock Matrix:\n";
		for(int i=0;i<src.size();i++){
			for(int j=0;j<src.size();j++)
				cout<<deadlock_matrix[i][j]<<" ";
			cout<<endl;
		}
		vector<vector<int> > collision_buckets;
		find_connected_components(deadlock_matrix,collision_buckets);
		print_collision_groups(collision_buckets);
		//generate frontier nodes from source and destination. The nodes will represent
		//collision zone boundary
		vector<int> frontier_start,frontier_end,frontier_start_indx,frontier_end_indx;
		frontier_start_indx=reachable_states_src[0];
		for(int i=1;i<reachable_states_src.size();i++){
			for(int j=0;j<src.size();j++){
				if(reachable_states_src[i][j]<frontier_start_indx[j])
					frontier_start_indx[j]=reachable_states_src[i][j];
			}
		}
		frontier_end_indx=reachable_states_dst[0];
		for(int i=1;i<reachable_states_dst.size();i++){
			for(int j=0;j<src.size();j++){
				if(reachable_states_dst[i][j]>frontier_end_indx[j])
					frontier_end_indx[j]=reachable_states_dst[i][j];
			}
		}
		for(int i=0;i<src.size();i++){
			frontier_start.push_back(paths[i][frontier_start_indx[i]]);
			frontier_end.push_back(paths[i][frontier_end_indx[i]]);
		}
		cout<<"Frontier-START:\n";
		for(int i=0;i<src.size();i++)
			cout<<frontier_start_indx[i]<<" ";
		cout<<"\nFrontier-END:\n";
		for(int i=0;i<src.size();i++)
			cout<<frontier_end_indx[i]<<" ";
		cout<<endl;

		//Composite planning for colliding robots
		for(int i=0;i<collision_buckets.size();i++){
			cout<<"Resolving collision group "<<i+1<<"...\n";
			vector<int> start,end,index;
			index=collision_buckets[i];		
			int category[index.size()];
			vector<int> paths_new[index.size()];
			for(int j=0;j<index.size();j++){
				start.push_back(frontier_start[index[j]]);
				end.push_back(frontier_end[index[j]]);
				category[j]=type[index[j]];
			}
			int res=findCompositePath_bidirectional(g1,g2,start,end,category,paths_new);
			if(res==0){
				cout<<"No path exist\n";
				return 1;
			}
			for(int j=0;j<index.size();j++){
				vector<int> temp;
				for(int k=0;k<frontier_start_indx[index[j]];k++)
					temp.push_back(paths[index[j]][k]);
				for(int k=0;k<paths_new[j].size();k++)
					temp.push_back(paths_new[j][k]);
				for(int k=frontier_end_indx[index[j]]+1;k<paths[index[j]].size();k++)
					temp.push_back(paths[index[j]][k]);
				paths[index[j]]=temp;
			}
		}
		//Print new paths
		for(int i=0;i<src.size();i++){
			cout<<"Path "<<i+1<<":\n";
			for(int j=0;j<paths[i].size();j++){
				cout<<paths[i][j]<<" ";
			}
			cout<<"\nPath length: "<<paths[i].size()-1<<endl;

		}
		// //Fixed path coordination with the new paths
		// cout<<"Found new paths.coordination in progress...\n";
		// unordered_map<string,pair<string,int> > cost;
		// memset(mask,false,sizeof(mask));
		// minConnectedState=getReachability(g1,g2,src,dst,paths,cost,mask,type);
		// if(cost.find(start)!=cost.end()){
		// 	string temp=start;
		// 	cout<<"Joint Path:"<<endl<<start<<endl;
		// 	while(temp!=goal){
		// 		cout<<cost[temp].first<<endl;
		// 		temp=cost[temp].first;
		// 	}
		// 	cout<<"SUCCESS! Renewed coordination cost="<<cost[start].second<<endl;
		// }
		// else{
			
		// 	cout<<"Trust me! No path exist\n";
		// 	return 1;
		// }
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
	string name="manipulators_thick_2";
	string fstr="/home/debojyoti/Dropbox/Navigation/"+name+"/csv/";
	string fstr1="/home/debojyoti/Dropbox/Navigation/"+name+"/images1/";
	string fstr2="/home/debojyoti/Dropbox/Navigation/"+name+"/images2/";
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
	// int type[]={1,1,1,1,1};
	int type[]={0,1};
	while(i<totalRun){
		// Tuple consists of one triangle followed by three circle instances
		//src=create_tuple(4,goodIndices1[rand()%goodIndices1.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()]);
		//dst=create_tuple(4,goodIndices1[rand()%goodIndices1.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()]);
		// src=create_tuple(5,82,1170,13,61,1290);
		// src=create_tuple(5,82,1170,13,3425,1290);
		// src=create_tuple(5,82,1170,13,42,1290);
		//dst=create_tuple(5,60,1130,1186,96,1246);	//3+2 robots deadlock
		//dst=create_tuple(5,60,1130,73,96,1294);	//collision free
		// dst=create_tuple(5,60,1130,1186,160,1246);	//2+2 robot deadlock
		//dst=create_tuple(5,103,160,1186,96);	//2 robot deadlock
		// dst=create_tuple(5,60,1130,1186,209,1246);

		// Manipulator arms
		// src=create_tuple(2,1992,174);
		// dst=create_tuple(2,5,3575);
		src=create_tuple(2,1238,188);
		dst=create_tuple(2,465,174);

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