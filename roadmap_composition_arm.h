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
vector<int> expand(vector<Graph> roadmaps,int size,int K,unordered_set<string> &taken,pqtype &pq,unordered_map<string,double> &inpq,unordered_map<string,vector<int> > &parents,int type[],bool mask[])
{
	// Returns extracted node from the priority queue
	pairtype temp=pq.top();
	pq.pop();
	vector<int> from=temp.second;
	double curdist=temp.first;
	if(taken.find(formKey(from))!=taken.end())
		return from;
	taken.insert(formKey(from));
	vector<pair<int,double> > nbr[size];
	set<pair<int,double> >::iterator it;
	for(int i=0;i<size;i++){
		for(it=roadmaps[type[i]].adjlst[from[i]].begin();it!=roadmaps[type[i]].adjlst[from[i]].end();it++){
			nbr[i].push_back(make_pair(it->first,it->second));
		}
		nbr[i].insert(nbr[i].begin(),make_pair(from[i],0));
	}
	// Finding neighbours of 'from' node in composite space by filtering out from composite nbrhood matrix
	vector<pairtype> selected,productMat;
	// Select Axial elements first
	for(int i=0;i<size;i++){
		vector<int> temp=from;
		set<pair<int,double> >::iterator j;
		for(int j=1;j<nbr[i].size();j++){
			temp[i]=nbr[i][j].first;
			selected.push_back(make_pair(nbr[i][j].second,temp));
		}
	}
	// Next step is to select best suitable elements from the rest of product matrix. We define
	// composite neighbour as follows: $Y \in nbr(X)$ if $Y(i)==nbr(X(i))$ or $X(i)$ itself for
	// all $i$.
	//Creating neighbourhood lattice in two 1D arrays...one for the nbr...other for the distance
	vector<vector<int> > indices;
	vector<float> l2sq_distance;
	for(int i=0;i<size;i++){
		if(indices.size()==0){
			for(int j=0;j<nbr[i].size();j++){
				vector<int> temp;
				temp.push_back(nbr[i][j].first);
				indices.push_back(temp);
				l2sq_distance.push_back(nbr[i][j].second*nbr[i][j].second);
			}
		}
		else{
			vector<vector<int> > temp_ind;
			vector<float> temp_distance;
			for(int j=0;j<indices.size();j++){
				vector<int> temp = indices[j];
				float distance = l2sq_distance[j];
				vector<int> temp_bck = indices[j];
				for(int k=0;k<nbr[i].size();k++){
					temp.push_back(nbr[i][k].first);
					// Dont consider axial elements
					if(i==size-1){
						int sum = 0;
						for(int l=0;l<i;l++){
							if(temp[l]==from[l])
								sum+=1;
						}
						if(sum==size-1){
							break;
						}
						if(sum==size-2 && temp[i]==from[i]){
							temp = temp_bck;
							continue;
						}
					}
					temp_ind.push_back(temp);
					temp_distance.push_back(distance+nbr[i][k].second*nbr[i][k].second);
					temp = temp_bck;
				}
			}
			indices = temp_ind;
			l2sq_distance = temp_distance;
		}
	}

	//Merge indices and l2sq_distance into product matrix
	for(int i=0;i<indices.size();i++){
		productMat.push_back(make_pair(sqrt(l2sq_distance[i]),indices[i]));
		// cout<<productMat.back().second[0]<<"-"<<productMat.back().second[1]<<endl;
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
int findCompositePath_bidirectional(vector<Graph> roadmaps,vector<int> src,vector<int> dst,int type[],vector<int> paths[])
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
			res=expand(roadmaps,src.size(),k,taken_src,pq_src,inpq_src,parents_src,type,mask);
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
			res=expand(roadmaps,src.size(),k,taken_dst,pq_dst,inpq_dst,parents_dst,type,mask);
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