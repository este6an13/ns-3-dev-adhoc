clear
git stash
git stash clear
git pull
rm pcap/p2p-*
rm positions.txt
./ns3 build
./ns3 run main
