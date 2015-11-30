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