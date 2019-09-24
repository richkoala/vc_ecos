function [mat,mat_nnz_idx] = func_decode_matrix_l(filename)
    fid = fopen(filename);
    dat = fscanf(fid,'%d%d%g',inf);
    dat = transpose(reshape(dat,3,[]));
    ROW = dat(end,1);
    COL = dat(end,1);

    mat = zeros(ROW,COL);
    for idx = 1:length(dat)-1
        mat(dat(idx,1),dat(idx,2))=dat(idx,3);
    end
    
    mat_nnz_idx = dat(end,2); 
    fclose all;
end