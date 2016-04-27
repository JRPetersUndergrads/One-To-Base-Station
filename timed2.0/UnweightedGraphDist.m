function distance = UnweightedGraphDist(edges, sources, destinations)
% take input of edges, sources and destinations return distance matrix
%rows correspond to sources
%columns correspond to destinations
distleft = false(size(edges));
distleft(sources,destinations) = true;%true if we still need to count the distance
count =1;
calc = edges;
distance = zeros(size(edges));
samepoint = eye(length(edges));
samepoint = samepoint == 0;
distleft = distleft.*samepoint; %get rid of points with same source/destination
while(sum(distleft(:))~=0) %while still have distances we are looking for
    %check to see if we have distance yet
    distance(distleft & calc >0) = count;
    %remove points from distleft
    distleft(calc > 0) = false;
    calc = calc*edges;
    count = count+1;
end
distance = distance(sources,destinations);
end

