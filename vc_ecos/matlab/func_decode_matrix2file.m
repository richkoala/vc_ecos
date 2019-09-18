function data_byte_len = func_decode_matrix2file(fid,fid_config)
    sop = fscanf(fid,'%d',4);
    eps_delta = fscanf(fid,'%g',2);
    
    fwrite(fid_config,sop,'uint32');
    fwrite(fid_config,eps_delta,'double');
    
    dat = fscanf(fid,'%d%d%g',inf);
    dat = transpose(reshape(dat,3,[]));
    ROW = dat(end,1);
    COL = dat(end,2);

    mat = zeros(ROW,COL);
    for idx = 1:length(dat)-1
        fwrite(fid_config,dat(idx,1),'uint32');
        fwrite(fid_config,dat(idx,2),'uint32');
        fwrite(fid_config,dat(idx,3),'double');
    end
    
    data_byte_len = sop(2);
    
end