
music FIRfilter(music *in,float *bz)
{
    int l;
    float lefta= 0.0;
	float righta=0.0;
    music temp;
    temp.left= 0.0;
    temp.right=0.0;
    for(l=0;l<N;l++)
    {
        lefta =lefta  + in[l].left * bz[l];
        righta=righta + in[l].right * bz[l];
    }
    

    temp.left=lefta;
	temp.right=righta;
    return temp;
}


music ellipsefilter(music *in,music *out,float *bz,float *az)
{
    int l;
    float lefta= 0.0;
	float righta=0.0;
    music temp;
    temp.left= 0.0;
    temp.right=0.0;
    for(l=0;l<N;l++)
    {
        lefta =lefta  + in[l].left * bz[l];
        righta=righta + in[l].right * bz[l];
    }
    
	l=0;
    for(l=0;l<(N-1);l++)
    {
        lefta  = lefta  + out[l].left  * (-1) * az[l+1];
        righta = righta + out[l].right * (-1) * az[l+1];
    }

    temp.left=lefta;
	temp.right=righta;
    return temp;
}
