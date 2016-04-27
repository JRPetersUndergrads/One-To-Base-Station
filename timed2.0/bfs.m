function [d] = bfs(A,u)
% BFS Compute breadth first search distances, times, and tree for a graph
%
% [d dt pred] = bfs(A,u) returns the distance (d) and the discover time
% (dt) for each vertex in the graph in a breadth first search
% starting from vertex u.
%   d = dt(i) = -1 if vertex i is not reachable from u
% pred is the predecessor array.  pred(i) = 0 if vertex (i)
% is in a component not reachable from u and i != u.
%
% [...] = bfs(A,u,v) stops the bfs when it hits the vertex v
%
% Example:
%   load_gaimc_graph('bfs_example.mat') % use the dfs example from Boost
%   d = bfs(A,1)
%
% See also DFS

% David F. Gleich
% Copyright, Stanford University, 2008-20098

% History
% 2008-04-13: Initial coding
A = sparse(A);[rp ci]=sparse_to_csr(A);
n=length(rp)-1;
numSources = length(u);
d=-1*ones(numSources,n);
for i = 1:length(u)
    sq=zeros(n,1); sqh=0; % search queue and search queue tail/head
    
    % start bfs at u
    sqt=1; sq(sqt)=u(i);
    d(i,u(i))=0;
    while sqt-sqh>0
        sqh=sqh+1; v=sq(sqh); % pop v off the head of the queue
        for ri=rp(v):rp(v+1)-1
            w=ci(ri);
            if d(i,w)<0
                sqt=sqt+1; sq(sqt)=w;
                d(i,w)=d(i,v)+1;
            end
        end
    end
end
