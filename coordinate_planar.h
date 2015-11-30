class compare{
public:
	bool operator()(pair<string,int> e1,pair<string,int> e2){
		return e1.second>e2.second;
	}
};
vector<int> getReachability(Graph g1,Graph g2,vector<int> src,vector<int> dst,vector<int> paths[],unordered_map<string,pair<string,int> > &cost,bool mask[],int type[],vector<vector<int> > &reachable_states)
{
	vector<vector<int> > minimal_set;
	int l[src.size()];
	vector<int> goal,goal_ind;
	cout<<"coordination in progress. Started at : ";
	showtime();
	for(int i=0;i<src.size();i++){
		if(!mask[i]){
			goal.push_back(paths[i].back());
			l[i]=paths[i].size();
		}
		else{
			goal.push_back(paths[i][0]);
			l[i]=1;
		}
		goal_ind.push_back(l[i]-1);
	}
	cost[formKey_indexed(goal,goal_ind)]=make_pair(formKey_indexed(goal,goal_ind),0);

	for(int p1=l[0]-1;p1>=0;p1--)
	for(int p2=l[1]-1;p2>=0;p2--)
	for(int p3=l[2]-1;p3>=0;p3--)
	for(int p4=l[3]-1;p4>=0;p4--)
	for(int p5=l[4]-1;p5>=0;p5--){
		vector<int> vfrom,vfrom_ind;
		vfrom.push_back(paths[0][p1]);vfrom.push_back(paths[1][p2]);vfrom.push_back(paths[2][p3]);vfrom.push_back(paths[3][p4]);vfrom.push_back(paths[4][p5]);
		vfrom_ind.push_back(p1);vfrom_ind.push_back(p2);vfrom_ind.push_back(p3);vfrom_ind.push_back(p4);vfrom_ind.push_back(p5);
		string from=formKey_indexed(vfrom,vfrom_ind);
		//check for static collision
		if(!isFeasible_static_multi(vfrom,mask,type)){	
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
			if(i1+i2+i3+i4+i5==0)
				continue;
			vector<int> vto,vto_ind;
			vto.push_back(paths[0][p1+i1]);vto.push_back(paths[1][p2+i2]);vto.push_back(paths[2][p3+i3]);vto.push_back(paths[3][p4+i4]);vto.push_back(paths[4][p5+i5]);
			vto_ind.push_back(p1+i1);vto_ind.push_back(p2+i2);vto_ind.push_back(p3+i3);vto_ind.push_back(p4+i4);vto_ind.push_back(p5+i5);
			string to=formKey_indexed(vto,vto_ind);
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
	reachable_states=minimal_set;
	vector<int> nonzero(src.size(),0);
	cout<<"States with no predecessor:\n";
	for(int i=0;i<minimal_set.size();i++){
		cout<<formKey(minimal_set[i])<<endl;
		for(int j=0;j<src.size();j++)
			nonzero[j]+=minimal_set[i][j];
	}
	cout<<"coordination ended at : ";
	showtime();
	return nonzero;
}