function v=unskew(S)
  tmp=S+S';
  tmp_norm=norm(tmp);
  if(length(S)~=3)
    error('unskew(): Invalid matrix size S');
  elseif(tmp_norm>1e-9) %eps(16)
    error('unskew(): Non-skew-symmetric matrix S');
  else
    v = zeros(3,1);
    v(1)=S(3,2);
    v(2)=S(1,3);
    v(3)=S(2,1);
  end
end
