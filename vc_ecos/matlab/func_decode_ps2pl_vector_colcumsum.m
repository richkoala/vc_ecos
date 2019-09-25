function s = func_decode_vector_colcumsum(filename)
    fid = fopen(filename);
    tmp = fscanf(fid,'%d',4);
    frame_id = tmp(1);
    len      = tmp(2);
    cnt      = tmp(3);
    iter_cnt = tmp(4);
    tmp = fscanf(fid,'%d',Inf);
    vector   = tmp(1:end-3);
    s = struct( ...
    'frame_id' , frame_id ...
    ,'len' , len ...
    ,'cnt' , cnt ...
    ,'iter_cnt' , iter_cnt ...
    ,'vector', vector ...
    );
    fclose all;
end