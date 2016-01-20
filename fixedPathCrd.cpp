#include<bits/stdc++.h>
#include<string>
#include<cmath>
#include<unordered_map>
#include<unordered_set>
#include<ctime>
using namespace std;
#define ROBOTICARM
#include "helper.h"
#ifdef ROBOTICARM
	#include "ds_arm.h"
	#include "collision_handler_arm.h"
	#include "coordinate_arm.h"
#else
	#include "ds_planar.h"
	#include "collision_handler_planar.h"
	#include "coordinate_planar.h"
#endif
#include "roadmap_composition.h"
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
				// cout<<"Corrected states with no successor:\n";
				for(int i=0;i<temp_rch.size();i++){
					for(int j=0;j<src.size();j++){
						temp_rch[i][j]=paths[j].size()-temp_rch[i][j]-1;
					}
					// cout<<formKey(temp_rch[i])<<endl;
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
		//Assign unresolved robots to proper collision group
		for(int i=0;i<unresolved.size();i++){
			for(int j=0;j<collision_buckets.size();j++){
				vector<int> group = collision_buckets[j];
				memset(mask,true,sizeof(mask));
				mask[unresolved[i]]=false;
				for(int k=0;k<group.size();k++)
					mask[group[k]]=false;
				unordered_map<string,pair<string,int> > tmp_cost;	//<from,<to,steps-to-take>>
				minConnectedState=getReachability(src,dst,paths_new,tmp_cost,mask,type,temp_rch);
				if(minConnectedState[unresolved[i]]>0){
					collision_buckets[j].push_back(unresolved[i]);
					break;
				}
			}
		}
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
			vector<int> start,end,index,start_src,end_dst;
			index=collision_buckets[i];		
			int category[index.size()];
			vector<int> paths_new[index.size()];
			for(int j=0;j<index.size();j++){
				start.push_back(frontier_start[index[j]]);
				start_src.push_back(src[index[j]]);
				end.push_back(frontier_end[index[j]]);
				end_dst.push_back(dst[index[j]]);
				category[j]=type[index[j]];
			}
			int res=findCompositePath_bidirectional(g1,g2,start,end,category,paths_new);
			if(res==0){
				// Failing to find path between frontiers, find path between the actual src and dst
				res=findCompositePath_bidirectional(roadmaps,start_src,end_dst,category,paths_new);
				if(res==0){
					cout<<"No path exist\n";
					return 1;
				}
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
	string name,fstr,fstr1,fstr2;
#ifdef ROBOTICARM
	name="manipulators_thick_2";
	fstr="/home/debojyoti/Dropbox/Navigation/"+name+"/csv/";
	fstr1="/home/debojyoti/Dropbox/Navigation/"+name+"/images1/";
	fstr2="/home/debojyoti/Dropbox/Navigation/"+name+"/images2/";
#else
	name="arena3";
	fstr="/home/debojyoti/Dropbox/Navigation/"+name+"/csv/";
	fstr1="/home/debojyoti/Dropbox/Navigation/"+name+"/triangle/";
	fstr2="/home/debojyoti/Dropbox/Navigation/"+name+"/circle/";
#endif
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
	vector<int> src,dst; //index starts from 0
	// // Random instances
	// ifstream fcollision1(fstr1+"collision.csv");
	// ifstream fcollision2(fstr2+"collision.csv");
	// vector<int> goodIndices1,goodIndices2;
	// i=0;
	// while(getline(fcollision1,line))
	// {
	// 	if(stoi(line)==0)
	// 		goodIndices1.push_back(i);
	// 	i++;
	// }
	// i=0;
	// while(getline(fcollision2,line))
	// {
	// 	if(stoi(line)==0)
	// 		goodIndices2.push_back(i);
	// 	i++;
	// }
	i=0;
	int totalRun=1;
	// srand(time(NULL));
	// int failure_count=0;
	// double heapSize_avg=0;
	// int heapSize_max=0;
	// double discardedEdgeRatio=0;
	// float average_clockTicks;
	// float max_clockTicks=0;
	lastTime=clock();
	while(i<totalRun){
#ifdef ROBOTICARM
		int type[]={0,1};
		// Manipulator arms
		// src=create_tuple(2,1992,174);
		// dst=create_tuple(2,5,3575);
		src=create_tuple(2,1238,188);
		dst=create_tuple(2,465,174);
#else
		int type[]={1,1,1,1,1};
		// Tuple consists of one triangle followed by three circle instances
		//src=create_tuple(4,goodIndices1[rand()%goodIndices1.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()]);
		//dst=create_tuple(4,goodIndices1[rand()%goodIndices1.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()],goodIndices2[rand()%goodIndices2.size()]);
		// src=create_tuple(5,82,1170,13,61,1290);
		// src=create_tuple(5,82,1170,13,3425,1290);
		src=create_tuple(5,82,1170,13,42,1290);
		//dst=create_tuple(5,60,1130,1186,96,1246);	//3+2 robots deadlock
		//dst=create_tuple(5,60,1130,73,96,1294);	//collision free
		// dst=create_tuple(5,60,1130,1186,160,1246);	//2+2 robot deadlock
		//dst=create_tuple(5,103,160,1186,96);	//2 robot deadlock
		dst=create_tuple(5,60,1130,1186,209,1246);
#endif

		// pair<double,double> spaceComplexity;	//<heapsize,ratio_of_discardedEdge>
		int ret=findJointShortestPath(g1,g2,src,dst,type);
		// if(ret==0)
		// 	continue;
		// else if(ret==1)
		// 	failure_count++;
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
	// t=clock()-t;
	// average_clockTicks=(float)t/totalRun;
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