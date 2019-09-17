function col_cumsum = func_decode_col_cumsum(filename)
    fid = fopen(filename);
    dat = fscanf(fid,'%d%d%g',inf);
    dat = transpose(reshape(dat,3,[]));
    ROW = dat(end,1);
    COL = dat(end,2);

    mat = zeros(ROW,COL);
    for idx = 1:length(dat)-1
        mat(dat(idx,1),dat(idx,2))=dat(idx,3);
    end
    
    mat_nnz_idx = dat(1:end-1,:); 
    col_cumsum = 
    fclose all;
end